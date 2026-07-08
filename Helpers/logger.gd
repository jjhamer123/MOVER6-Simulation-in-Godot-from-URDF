extends Node
class_name BaseLinkLogger
 
@export var godot_robot: GodotRobot
@export var log_interval_sec: float = 1
@export var write_to_file: bool = true
 
var _end_body: RigidBody3D
var _timer: float = 0.0
var _start_transform: Transform3D
 
func _find_rigid_bodies(node: Node) -> Array[RigidBody3D]:
	var result: Array[RigidBody3D] = []
	for child in node.get_children():
		if child is RigidBody3D:
			result.append(child)
		result.append_array(_find_rigid_bodies(child))
	return result
 
func _ready() -> void:
 
	var bodies: Array[RigidBody3D] = _find_rigid_bodies(godot_robot)
 
	_end_body = bodies[-1]
	_start_transform = _end_body.global_transform

func _physics_process(delta: float) -> void:
	if not _end_body:
		return
	_timer += delta
	if _timer < log_interval_sec:
		return
	_timer = 0.0
 
	var t: Transform3D = _end_body.global_transform
	var pos: Vector3 = t.origin
	var rot_deg: Vector3 = t.basis.get_euler() * (180.0 / PI)
 
	print("END LINK  t=%.2fs  pos=(%.4f, %.4f, %.4f)  rot(deg)=(%.2f, %.2f, %.2f) " % [
		Time.get_ticks_msec() / 1000.0, pos.x, pos.y, pos.z, rot_deg.x, rot_deg.y, rot_deg.z
	])
