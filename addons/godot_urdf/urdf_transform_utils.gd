class_name URDFTransformUtils
extends RefCounted

# Pure rotation matrix: ROS (X-fwd, Y-left, Z-up) -> Godot (X-right, Y-up, Z-back)
const ROS_TO_GODOT_BASIS := Basis(
	Vector3(0, 0, -1), # ROS X -> Godot -Z
	Vector3(-1, 0, 0), # ROS Y -> Godot -X
	Vector3(0, 1, 0)   # ROS Z -> Godot +Y
)

static func ros_vec_to_godot(v: Vector3) -> Vector3:
	return ROS_TO_GODOT_BASIS * v

static func convert_urdf_origin(xyz: Vector3, rpy: Vector3) -> Transform3D:
	var pos := ros_vec_to_godot(xyz)
	# ROS uses intrinsic Z-Y-X Euler order for RPY
	var ros_basis := Basis.from_euler(Vector3(rpy.x, rpy.y, rpy.z), EulerOrder.EULER_ORDER_ZYX)
	# Similarity transformation: M * R_ros * M^-1
	var godot_basis := ROS_TO_GODOT_BASIS * ros_basis * ROS_TO_GODOT_BASIS.transposed()
	return Transform3D(godot_basis, pos)

static func get_joint_rotation_basis(axis_ros: Vector3, angle_rad: float) -> Basis:
	# 1. Transform URDF axis vector into Godot coordinate space
	var godot_axis := ros_vec_to_godot(axis_ros).normalized()
	# 2. Construct a pure rotation around the converted axis vector
	return Basis(godot_axis, angle_rad)
