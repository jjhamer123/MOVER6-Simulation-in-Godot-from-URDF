class_name URDF6DOFJoint3D
extends Generic6DOFJoint3D

@export var joint: URDFJoint

func update_joint(
		godot_robot: GodotRobot,
		owner: Node3D,
		joint: URDFJoint) -> void:
	godot_robot.add_child(self)
	self.owner = owner

	self.joint = joint
	self.name = "joint_" + joint.name
	self.exclude_nodes_from_collision = true

	# lock all movement and rotation (fixed = default)
	for axis_func in ["set_param_x", "set_param_y", "set_param_z"]:
		self.call(
			axis_func, Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
		self.call(
			axis_func, Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
	for axis_func in ["set_param_x", "set_param_y"]:
		self.call(
			axis_func, Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, 0.0)
		self.call(
			axis_func, Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, 0.0)

	if joint.type == "revolute" and joint.limit:
		# Limited hinge
		self.set_param_z(
			Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, joint.limit.lower)
		self.set_param_z(
			Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, joint.limit.upper)

	elif joint.type == "continuous":
		# Unlimited hinge. Jolt's 6DOF splits rotation into "twist" about local X and
		# "swing" about Y/Z; a free Z axis jams near +/-180 deg as soon as the chain
		# sags a little. So a continuous joint spins about local X instead (the basis
		# is turned below so X = the URDF axis) - see hinge_axis().
		self.set_meta("hinge_axis", "x")
		self.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, 0.0)
		self.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, 0.0)
		self.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, 1.0)
		self.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, 0.0)
		self.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, false)

	elif joint.type == "prismatic":
		# Slider along the joint's local Z (the basis is rotated below so that
		# local Z = the URDF <axis>). Limits are relative to the URDF zero pose.
		var lo := 0.0
		var hi := 0.0
		if joint.limit:
			lo = joint.limit.lower
			hi = joint.limit.upper
		self.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, lo)
		self.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, hi)
		# no twist about the slide axis
		self.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, 0.0)
		self.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, 0.0)
		# linear motor holds / drives the slide (velocity 0 = hold).
		# effort="0" in the URDF = a passive slide: no motor, only friction (if given).
		var lin_force := 50.0
		if joint.limit:
			lin_force = joint.limit.effort
		if lin_force <= 0.0 and joint.dynamics and joint.dynamics.friction > 0.0:
			lin_force = joint.dynamics.friction
		self.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_MOTOR, lin_force > 0.0)
		self.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_MOTOR_TARGET_VELOCITY, 0.0)
		self.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_MOTOR_FORCE_LIMIT, max(lin_force, 0.0))

	_hinge_flag(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, true)
	_hinge_param(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, 0.0)

	if joint.dynamics:
		if joint.dynamics.friction > 0:
			_hinge_param(
				Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT,
				joint.dynamics.friction)
		if joint.dynamics.damping > 0:
			# Damping is not supported by Jolt Physics, use Springinstead
			# if joint.dynamics.damping > 0:
			# 	godot_joint.set_param_z(
			# 		Generic6DOFJoint3D.PARAM_ANGULAR_DAMPING,
			# 		joint.dynamics.damping)
			_hinge_flag(
				Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_SPRING, true)
			_hinge_param(
				Generic6DOFJoint3D.PARAM_ANGULAR_SPRING_DAMPING,
				joint.dynamics.damping)

	if joint.limit and joint.limit.effort > 0:
		# If friction is already using the motor, we ensure the effort is
		# at least as high as friction
		var max_force = max(
			joint.limit.effort,
			joint.dynamics.friction if joint.dynamics else 0.0)
		_hinge_param(
			Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, max_force)

	if _hinge_get(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT) <= 0.0:
		_hinge_param(
			Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, 150.0)

	# A passive hinge (effort="0", e.g. a free bearing) gets no motor at all.
	# Bearing friction is tiny, and a zero-speed motor on a free axis makes Jolt
	# stick when the hinge passes +/-180 deg.
	if joint.type in ["continuous", "revolute"] and (not joint.limit or joint.limit.effort <= 0.0):
		_hinge_flag(Generic6DOFJoint3D.FLAG_ENABLE_MOTOR, false)

	# apply transform to position and rotate hinge
	var child_transform = godot_robot.get_rel_transform(joint.child)
	self.position = child_transform.origin
	var base_basis = child_transform.basis
	var urdf_axis = joint.axis.normalized()
	var local_axis := Vector3(1, 0, 0) if hinge_axis() == "x" else Vector3(0, 0, 1)
	var axis_rotation = Quaternion(local_axis, urdf_axis)
	self.transform.basis = base_basis * Basis(axis_rotation)


## Which local axis the joint moves about/along: "z" (default) or "x" (continuous).
func hinge_axis() -> String:
	return str(get_meta("hinge_axis", "z"))


func _hinge_param(param: int, value: float) -> void:
	if hinge_axis() == "x":
		set_param_x(param, value)
	else:
		set_param_z(param, value)


func _hinge_flag(flag: int, value: bool) -> void:
	if hinge_axis() == "x":
		set_flag_x(flag, value)
	else:
		set_flag_z(flag, value)


func _hinge_get(param: int) -> float:
	return get_param_x(param) if hinge_axis() == "x" else get_param_z(param)
