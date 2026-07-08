extends Node
class_name RobotJointController

@export var godot_robot: GodotRobot

var _joints: Dictionary[String, Dictionary] = {}

func _ready() -> void:
	await get_tree().physics_frame
	_setup()

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
		}

	print("RobotJointController: ready, tracking ", _joints.size(), " joint(s): ", _joints.keys())

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
