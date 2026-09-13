extends Node
class_name RobotJointController

@export var godot_robot: GodotRobot

## If a link's URDF had no <inertial> tag at all, estimate a real mass from
## its collision geometry instead of leaving it at Godot's flat default of
## 1kg (this is what caused the overshoot/instability on Mover6 before link
## masses were manually set).
@export var estimate_missing_mass: bool = true
@export var default_link_density_kg_m3: float = 1200.0
## AABB-based mesh volume estimate is inherently an overestimate (a hull
## rarely fills its bounding box) - this knocks it down to a more realistic
## fraction. Tune per-robot if link masses come out too heavy/light.
@export var mesh_collider_fill_factor: float = 0.55

## Optional hand-tuned masses, keyed by the link's actual URDF name (not
## joint name). Only needed if the auto-estimate above isn't good enough for
## a particular arm - leave empty to just use geometry estimation.
@export var link_mass_overrides: Dictionary[String, float] = {}

var _joints: Dictionary[String, Dictionary] = {}
var _link_names: Dictionary[String, String] = {}  # joint name -> child link name

func _ready() -> void:
	await get_tree().physics_frame
	reinitialize()

## Rebuilds the joint/link tracking from godot_robot. Call this after
## swapping in a new GodotRobot (e.g. loading a different URDF) - it's not
## just a one-time _ready() step.
func reinitialize() -> void:
	if not godot_robot:
		push_warning("RobotJointController: no godot_robot assigned")
		return
	_joints.clear()
	_link_names.clear()
	_setup()
	apply_link_masses()
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

		var world_axis: Vector3 = joint_node.global_transform.basis.z.normalized()
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
		}
		_link_names[joint.name] = joint.child


func apply_link_masses() -> void:
	for joint_name in _joints.keys():
		var link_name: String = _link_names.get(joint_name, "")
		var body_b: RigidBody3D = _joints[joint_name]["body_b"]

		if link_mass_overrides.has(link_name):
			body_b.mass = link_mass_overrides[link_name]
			continue

		if not estimate_missing_mass:
			continue

		# URDFRigidBody3D carries the source URDFLink directly - if it has no
		# <inertial> tag at all, `link.inertial` is null and godot_urdf left
		# body.mass at Godot's flat default. That's the real "missing" signal
		# (checking mass == 1.0 would also wrongly match a URDF that
		# genuinely specifies mass="1.0").
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
		# godot_urdf builds mesh colliders as ConvexPolygonShape3D via
		# Mesh.create_convex_shape() - almost every real URDF link (any STL/
		# DAE mesh) lands here. get_debug_mesh()'s AABB is a bounding-box
		# estimate, not the true hull volume, so it's scaled down by
		# mesh_collider_fill_factor to avoid badly overestimating mass.
		var aabb: AABB = shape.get_debug_mesh().get_aabb()
		var s: Vector3 = aabb.size * scale
		return s.x * s.y * s.z * mesh_collider_fill_factor
	return 0.0


#CONTROL THE ROBOT
func set_joint_velocity(joint_name: String, velocity_rad_s: float) -> void:
	if not _joints.has(joint_name):
		push_warning("RobotJointController: unknown jont '%s'" % joint_name)
		return
	var joint_node: Generic6DOFJoint3D = _joints[joint_name]["node"]
	joint_node.set_param_z(
		Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, velocity_rad_s)

# joint angle output
func get_joint_angle(joint_name: String) -> float:
	if not _joints.has(joint_name):
		push_warning("RobotJointController: unknown jont '%s'" % joint_name)
		return 0.0

	var data: Dictionary = _joints[joint_name]
	var body_a: RigidBody3D = data["body_a"]
	var body_b: RigidBody3D = data["body_b"]
	var axis: Vector3 = data["axis_local_to_a"]


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
	if not _joints.has(joint_name):
		return Vector2(-INF, INF)
	var data: Dictionary = _joints[joint_name]
	# Only "revolute" joints have a real angular limit. "continuous" (and
	# anything else) is unlimited - Godot represents that internally as
	# lower_limit=1.0, upper_limit=0.0 (lower > upper = disabled), which
	# would otherwise get misread as a genuine limit of (1.0, 0.0).
	if data.get("type", "") != "revolute":
		return Vector2(-INF, INF)
	var joint_node: Generic6DOFJoint3D = data["node"]
	var lower = joint_node.get_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT)
	var upper = joint_node.get_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT)
	return Vector2(lower, upper)
