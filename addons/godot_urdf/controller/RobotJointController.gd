extends Node
class_name RobotJointController

@export var godot_robot: GodotRobot

## if link's URDF had no <inertial> tag at all, estimate a real mass from
@export var estimate_missing_mass: bool = true
@export var default_link_density_kg_m3: float = 1200.0

@export var mesh_collider_fill_factor: float = 0.55

#Custom masses
@export var link_mass_overrides: Dictionary[String, float] = {}

## Robot links never collide with each other (they still hit the floor/paper).
## Neighbouring links are already excluded by the joints; this also covers
## links that are 2+ joints apart (e.g. a support bar overlapping the next layer).
@export var disable_self_collision: bool = false
## Jolt joints go soft when a light link carries a much heavier one (e.g. a
## 0.1 kg bar holding a 0.5 kg motor). Raising every link to at least this mass
## keeps long chains stiff. 0 = use the URDF masses unchanged.
@export var min_link_mass: float = 0.0
## Kinematic mode: links are frozen and posed exactly from the joint values
## every physics frame (no joint sag/wobble, no gravity or reaction torques).
## Same API as physics mode, so ros_2_arm.gd etc. work unchanged.
## Joints whose type is "fixed" stay at 0; continuous/revolute/prismatic move.
@export var kinematic_mode: bool = false
## Added to a joint's value on the way out (get_joint_angle, get_joint_limits,
## /joint_states) and removed on the way in (set_joint_angle). Use it when the
## URDF loaded here has a different zero than the one ROS/MATLAB uses, e.g.
## {"clampB1": 0.07} so clampB values are the true arm radius in metres.
@export var joint_offsets: Dictionary[String, float] = {}
## Kinematic mode: passive slides held by a fixed guide (e.g. a support bar whose
## ends run in frame-mounted tracks). Each frame the slide is set so its child
## link stays level with this global point along the slide axis.
## {"support2": Vector3(...)}. URDF <mimic> joints are followed automatically.
@export var guided_slides: Dictionary[String, Vector3] = {}
## Kinematic mode: a prismatic joint called support<N> whose robot also has links
## track<N>_L and track<N>_R (the Robot Art URDFs) is guided automatically,
## centred between those two tracks. Entries in Guided Slides win.
@export var auto_track_guides: bool = true

## Position control (used by set_joint_angle): a P-controller drives the
## joint motor towards the target angle each physics frame.
@export var position_kp: float = 10.0
@export var position_max_velocity: float = 3.0
## Max slide speed (m/s) used when driving a prismatic joint to a target.
@export var position_max_linear_velocity: float = 0.1

var _joints: Dictionary[String, Dictionary] = {}
var _link_names: Dictionary[String, String] = {}  # joint name -> child link name
## Force a joint's motor direction (+1 or -1) instead of learning it
@export var motor_direction_overrides: Dictionary[String, float] = {}

var _targets: Dictionary[String, float] = {}      # joint name -> target angle (rad)
var _last_cmd: Dictionary[String, float] = {}     # last motor velocity sent
var _prev_angle: Dictionary[String, float] = {}
var _motor_dir: Dictionary[String, float] = {}
var _dir_votes: Dictionary[String, int] = {}
var _dir_locked: Dictionary[String, bool] = {}
var _kin_q: Dictionary[String, float] = {}       # kinematic mode joint values
var _kin_v: Dictionary[String, float] = {}       # kinematic mode joint velocities
var _kin_order: Array[String] = []               # joints sorted parent -> child
var _auto_guides_pending := true

func _ready() -> void:
	await get_tree().physics_frame
	reinitialize()

#New URDF Builder

func reinitialize() -> void:
	if not godot_robot:
		push_warning("RobotJointController: no godot_robot assigned")
		return
	if godot_robot.links.is_empty() and godot_robot.has_method("rebuild_caches"):
		godot_robot.rebuild_caches()  # robot saved in a scene rather than built at runtime
	_joints.clear()
	_link_names.clear()
	_setup()
	apply_link_masses()
	if disable_self_collision:
		_disable_self_collision()
	if kinematic_mode:
		_setup_kinematic()
	print("RobotJointController: ready, tracking ", _joints.size(), " joint(s): ", _joints.keys())

func _setup() -> void:
	for joint in godot_robot.urdf.joints:
		var joint_node := godot_robot.get_node_or_null("joint_" + joint.name) as Generic6DOFJoint3D
		if not joint_node:
			continue
		var body_a := joint_node.get_node_or_null(joint_node.node_a) as RigidBody3D
		var body_b := joint_node.get_node_or_null(joint_node.node_b) as RigidBody3D
		if not body_a or not body_b:
			continue

		# continuous joints spin about the node's local X (see urdf_6dof_joint3d.gd), others about Z
		var on_x: bool = str(joint_node.get_meta("hinge_axis", "z")) == "x"
		var world_axis: Vector3 = (joint_node.global_transform.basis.x if on_x
				else joint_node.global_transform.basis.z).normalized()
		var axis_local_to_a: Vector3 = (
				body_a.global_transform.basis.inverse() * world_axis).normalized()

		_joints[joint.name] = {
			"node": joint_node,
			"body_a": body_a,
			"body_b": body_b,
			"axis_local_to_a": axis_local_to_a,
			"rest_basis_a": body_a.global_transform.basis,
			"rest_basis_b": body_b.global_transform.basis,
			"type": joint.type,
			# <mimic joint=... multiplier=... offset=...> (needs the patched parser)
			"mimic": str(joint.get("mimic_joint")) if joint.get("mimic_joint") else "",
			"mimic_mult": float(joint.get("mimic_multiplier")) if joint.get("mimic_multiplier") != null else 1.0,
			"mimic_off": float(joint.get("mimic_offset")) if joint.get("mimic_offset") != null else 0.0,
			# prismatic: body_b's rest position in body_a's frame, to measure the slide
			"rest_pos_in_a": body_a.global_transform.affine_inverse() * body_b.global_position,
		}
		_link_names[joint.name] = joint.child


func _setup_kinematic() -> void:
	_auto_guides_pending = true
	_kin_q.clear()
	_kin_v.clear()
	_kin_order.clear()
	for joint_name in _joints.keys():
		var d: Dictionary = _joints[joint_name]
		var a: RigidBody3D = d["body_a"]
		var b: RigidBody3D = d["body_b"]
		var jn: Node3D = d["node"]
		d["pivot_in_a"] = a.global_transform.affine_inverse() * jn.global_position
		d["rel_rest"] = a.global_transform.affine_inverse() * b.global_transform
		_kin_q[joint_name] = 0.0
		_kin_v[joint_name] = 0.0
	for link in godot_robot.links.values():
		if link is RigidBody3D:
			link.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
			link.freeze = true
	# order joints so a parent body is always posed before its children
	var posed: Dictionary = {}
	var children: Dictionary = {}
	for joint_name in _joints.keys():
		children[_joints[joint_name]["body_b"]] = true
	for joint_name in _joints.keys():
		var a = _joints[joint_name]["body_a"]
		if not children.has(a):
			posed[a] = true
	var remaining: Array = _joints.keys()
	while not remaining.is_empty():
		var progressed := false
		for joint_name in remaining.duplicate():
			if posed.has(_joints[joint_name]["body_a"]):
				_kin_order.append(joint_name)
				posed[_joints[joint_name]["body_b"]] = true
				remaining.erase(joint_name)
				progressed = true
		if not progressed:
			_kin_order.append_array(remaining)
			break


func _auto_guides() -> void:
	if not auto_track_guides:
		return
	for joint_name in _joints.keys():
		if guided_slides.has(joint_name) or not joint_name.begins_with("support") \
				or _joints[joint_name].get("type", "") != "prismatic":
			continue
		var suffix: String = joint_name.substr("support".length())
		var l = godot_robot.links.get("track%s_L" % suffix)
		var r = godot_robot.links.get("track%s_R" % suffix)
		if l is Node3D and r is Node3D:
			guided_slides[joint_name] = (l.global_position + r.global_position) * 0.5


func _kinematic_step(delta: float) -> void:
	if auto_track_guides and _auto_guides_pending:
		_auto_guides_pending = false
		_auto_guides()
	for joint_name in _kin_order:
		var d: Dictionary = _joints[joint_name]
		var type: String = d.get("type", "fixed")
		var src: String = d.get("mimic", "")
		if type != "fixed" and src != "" and _kin_q.has(src):
			_kin_q[joint_name] = _kin_q[src] * d["mimic_mult"] + d["mimic_off"]
		elif type == "prismatic" and guided_slides.has(joint_name):
			# slide so the child stays level with the guide point along the slide axis
			var a0: RigidBody3D = d["body_a"]
			var ax_w: Vector3 = (a0.global_transform.basis * d["axis_local_to_a"]).normalized()
			var origin_w: Vector3 = a0.global_transform * d["rel_rest"].origin
			var qg: float = (guided_slides[joint_name] - origin_w).dot(ax_w)
			var lim_g := _raw_limits(joint_name)
			_kin_q[joint_name] = clamp(qg, lim_g.x, lim_g.y) if lim_g.x <= lim_g.y else qg
		elif type != "fixed":
			var q: float = _kin_q[joint_name] + _kin_v[joint_name] * delta
			if type == "revolute" or type == "prismatic":
				var lim := _raw_limits(joint_name)
				if is_finite(lim.x) and is_finite(lim.y) and lim.x <= lim.y:
					q = clamp(q, lim.x, lim.y)
			_kin_q[joint_name] = q
		var a: RigidBody3D = d["body_a"]
		var b: RigidBody3D = d["body_b"]
		var axis: Vector3 = d["axis_local_to_a"]
		var q2: float = _kin_q[joint_name]
		var motion := Transform3D.IDENTITY
		if type == "prismatic":
			motion = Transform3D(Basis.IDENTITY, axis * q2)
		elif type != "fixed":
			var pivot: Vector3 = d["pivot_in_a"]
			motion = Transform3D(Basis.IDENTITY, pivot) \
					* Transform3D(Basis(axis, q2), Vector3.ZERO) \
					* Transform3D(Basis.IDENTITY, -pivot)
		b.global_transform = a.global_transform * motion * d["rel_rest"]


func _disable_self_collision() -> void:
	var bodies: Array[PhysicsBody3D] = []
	for link in godot_robot.links.values():
		if link is PhysicsBody3D:
			bodies.append(link)
	for i in bodies.size():
		for j in range(i + 1, bodies.size()):
			bodies[i].add_collision_exception_with(bodies[j])


func apply_link_masses() -> void:
	for joint_name in _joints.keys():
		var link_name: String = _link_names.get(joint_name, "")
		var body_b: RigidBody3D = _joints[joint_name]["body_b"]
		if min_link_mass > 0.0 and body_b.mass < min_link_mass and not link_mass_overrides.has(link_name):
			body_b.mass = min_link_mass

		if link_mass_overrides.has(link_name):
			body_b.mass = link_mass_overrides[link_name]
			continue

		if not estimate_missing_mass:
			continue

		var urdf_link: URDFLink = (body_b as URDFRigidBody3D).link if body_b is URDFRigidBody3D else null
		if urdf_link and urdf_link.inertial:
			continue

		var estimated := _estimate_mass_from_collision(body_b)
		if estimated > 0.0:
			body_b.mass = estimated
			print("RobotJointController: '%s' had no URDF <inertial>, estimated %.3f kg from geometry" % [link_name, estimated])


func _estimate_mass_from_collision(body: RigidBody3D) -> float:
	var volume := 0.0
	for child in body.get_children():
		if child is CollisionShape3D and child.shape:
			volume += _shape_volume(child.shape, child.scale)
	return volume * default_link_density_kg_m3


func _shape_volume(shape: Shape3D, scale: Vector3) -> float:
	if shape is BoxShape3D:
		var s: Vector3 = shape.size * scale
		return s.x * s.y * s.z
	elif shape is SphereShape3D:
		var r: float = shape.radius * scale.x
		return (4.0 / 3.0) * PI * r * r * r
	elif shape is CapsuleShape3D:
		var r: float = shape.radius * scale.x
		var h: float = shape.height * scale.y
		return PI * r * r * h + (4.0 / 3.0) * PI * r * r * r
	elif shape is CylinderShape3D:
		var r: float = shape.radius * scale.x
		var h: float = shape.height * scale.y
		return PI * r * r * h
	elif (shape is ConvexPolygonShape3D or shape is ConcavePolygonShape3D) \
			and shape.has_method("get_debug_mesh"):
		var aabb: AABB = shape.get_debug_mesh().get_aabb()
		var s: Vector3 = aabb.size * scale
		return s.x * s.y * s.z * mesh_collider_fill_factor
	return 0.0


#CONTROL ROBOT
func set_joint_velocity(joint_name: String, velocity_rad_s: float) -> void:
	if not _joints.has(joint_name):
		push_warning("RobotJointController: unknown jont '%s'" % joint_name)
		return
	_targets.erase(joint_name)  # velocity command overrides any position target
	if kinematic_mode:
		_kin_v[joint_name] = velocity_rad_s
		_last_cmd[joint_name] = velocity_rad_s
		return
	_set_motor_velocity(joint_name, velocity_rad_s)

## Drive a joint to a target angle (rad). Held until a velocity command
## or clear_joint_targets() replaces it.
func set_joint_angle(joint_name: String, angle_rad: float) -> void:
	if not _joints.has(joint_name):
		push_warning("RobotJointController: unknown jont '%s'" % joint_name)
		return
	angle_rad -= joint_offsets.get(joint_name, 0.0)
	var limits := _raw_limits(joint_name)
	if is_finite(limits.x) and is_finite(limits.y):
		angle_rad = clamp(angle_rad, limits.x, limits.y)
	_targets[joint_name] = angle_rad

func clear_joint_targets() -> void:
	for joint_name in _targets.keys():
		_set_motor_velocity(joint_name, 0.0)
	_targets.clear()

func is_prismatic(joint_name: String) -> bool:
	return _joints.has(joint_name) and _joints[joint_name].get("type", "") == "prismatic"

func _set_motor_velocity(joint_name: String, velocity_rad_s: float) -> void:
	if kinematic_mode:
		_kin_v[joint_name] = velocity_rad_s
		_last_cmd[joint_name] = velocity_rad_s
		return
	var joint_node: Generic6DOFJoint3D = _joints[joint_name]["node"]
	if is_prismatic(joint_name):
		# for a prismatic joint the "velocity" is m/s along the slide
		joint_node.set_param_z(
			Generic6DOFJoint3D.PARAM_LINEAR_MOTOR_TARGET_VELOCITY, velocity_rad_s)
	elif str(joint_node.get_meta("hinge_axis", "z")) == "x":
		joint_node.set_param_x(
			Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, velocity_rad_s)
	else:
		joint_node.set_param_z(
			Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, velocity_rad_s)
	_last_cmd[joint_name] = velocity_rad_s

## +1 if a positive motor velocity makes get_joint_angle() increase, -1 if it
## decreases. Learned automatically from how each joint actually moves.
func get_motor_direction(joint_name: String) -> float:
	if motor_direction_overrides.has(joint_name):
		return motor_direction_overrides[joint_name]
	if is_prismatic(joint_name) or kinematic_mode:
		return 1.0  # slide is measured along the motor's own axis
	return _motor_dir.get(joint_name, 1.0)

func _learn_motor_directions(delta: float) -> void:
	for joint_name in _joints.keys():
		var angle := _raw_angle(joint_name)
		if _prev_angle.has(joint_name) and not _dir_locked.get(joint_name, false) \
				and not is_prismatic(joint_name) \
				and not motor_direction_overrides.has(joint_name):
			var rate := ((angle - _prev_angle[joint_name]) if is_prismatic(joint_name) \
					else angle_difference(_prev_angle[joint_name], angle)) / delta
			var cmd: float = _last_cmd.get(joint_name, 0.0)
			if absf(cmd) > 0.05 and absf(rate) > 0.02:
				var vote := 1 if signf(rate) == signf(cmd) else -1
				_dir_votes[joint_name] = _dir_votes.get(joint_name, 0) + vote
				if absi(_dir_votes[joint_name]) >= 5:
					_motor_dir[joint_name] = signf(_dir_votes[joint_name])
					_dir_locked[joint_name] = true
					print("RobotJointController: '%s' motor direction = %+d" \
							% [joint_name, int(_motor_dir[joint_name])])
		_prev_angle[joint_name] = angle

func _physics_process(delta: float) -> void:
	if kinematic_mode and not _kin_order.is_empty():
		for joint_name in _targets.keys():
			var e := (_targets[joint_name] - _kin_q[joint_name]) if is_prismatic(joint_name) \
					else angle_difference(_raw_angle(joint_name), _targets[joint_name])
			var vm := position_max_linear_velocity if is_prismatic(joint_name) else position_max_velocity
			# move at up to vm, but never overshoot within one frame
			_kin_v[joint_name] = clamp(e / delta, -vm, vm)
		_kinematic_step(delta)
		return
	_learn_motor_directions(delta)
	for joint_name in _targets.keys():
		var err := (_targets[joint_name] - _raw_angle(joint_name)) if is_prismatic(joint_name) \
				else angle_difference(_raw_angle(joint_name), _targets[joint_name])
		var vmax := position_max_linear_velocity if is_prismatic(joint_name) else position_max_velocity
		var v: float = clamp(position_kp * err, -vmax, vmax)
		_set_motor_velocity(joint_name, v * get_motor_direction(joint_name))

# joint angle output
func get_joint_angle(joint_name: String) -> float:
	return _raw_angle(joint_name) + joint_offsets.get(joint_name, 0.0)


func _raw_angle(joint_name: String) -> float:
	if not _joints.has(joint_name):
		push_warning("RobotJointController: unknown jont '%s'" % joint_name)
		return 0.0

	var data: Dictionary = _joints[joint_name]
	if kinematic_mode and _kin_q.has(joint_name):
		var kq: float = _kin_q[joint_name]
		return kq if is_prismatic(joint_name) or data.get("type", "") == "revolute" else wrapf(kq, -PI, PI)
	var body_a: RigidBody3D = data["body_a"]
	var body_b: RigidBody3D = data["body_b"]
	var axis: Vector3 = data["axis_local_to_a"]

	if data.get("type", "") == "prismatic":
		# slide distance (m) from the URDF zero pose, along the joint axis
		var pos_in_a: Vector3 = body_a.global_transform.affine_inverse() * body_b.global_position
		return (pos_in_a - data["rest_pos_in_a"]).dot(axis)

	var rel_now: Basis = body_a.global_transform.basis.inverse() * body_b.global_transform.basis
	var rel_rest: Basis = data["rest_basis_a"].inverse() * data["rest_basis_b"]
	var delta: Basis = rel_rest.inverse() * rel_now

	var ref_dir: Vector3 = axis.cross(Vector3.UP)
	if ref_dir.length() < 0.001:
		ref_dir = axis.cross(Vector3.RIGHT)
	ref_dir = ref_dir.normalized()

	return ref_dir.signed_angle_to(delta * ref_dir, axis)

## for logging or ROS driver
func get_all_angles() -> Dictionary[String, float]:
	var anglearr: Dictionary[String, float] = {}
	for joint_name in _joints.keys():
		anglearr[joint_name] = get_joint_angle(joint_name)
	return anglearr

## get the joint names 
func get_joint_names() -> Array[String]:
	var namesarr: Array[String] = []
	namesarr.assign(_joints.keys())
	return namesarr
	
func get_joint_limits(joint_name: String) -> Vector2:
	var off: float = joint_offsets.get(joint_name, 0.0)
	return _raw_limits(joint_name) + Vector2(off, off)


func _raw_limits(joint_name: String) -> Vector2:
	if not _joints.has(joint_name):
		return Vector2(-INF, INF)
	var data: Dictionary = _joints[joint_name]
	if data.get("type", "") == "prismatic":
		var jn: Generic6DOFJoint3D = data["node"]
		return Vector2(jn.get_param_z(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT),
				jn.get_param_z(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT))
	if data.get("type", "") != "revolute":
		return Vector2(-INF, INF)
	var joint_node: Generic6DOFJoint3D = data["node"]
	var lower = joint_node.get_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT)
	var upper = joint_node.get_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT)
	return Vector2(lower, upper)
