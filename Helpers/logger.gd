extends Node
class_name BaseLinkLogger
 
@export var godot_robot: GodotRobot
@export var log_interval_sec: float = 1
@export var write_to_file: bool = true
## Optional: force which link to track (its URDF link name). Leave blank to
## auto-pick the deepest leaf link (the usual end-effector for a serial arm).
## Set this explicitly for grippers/branching arms where auto-pick might grab
## the wrong tip.
@export var end_link_name: String = ""
 
var _end_body: RigidBody3D
var _timer: float = 0.0
var _start_transform: Transform3D
 
func _ready() -> void:
	reinitialize()

## Re-finds the tracked end-link. Call this after swapping in a new
## GodotRobot - it's not just a one-time _ready() step.
func reinitialize() -> void:
	if not godot_robot:
		push_warning("BaseLinkLogger: no godot_robot assigned")
		return

	var link_name := end_link_name
	if link_name.is_empty():
		link_name = godot_robot.get_deepest_leaf_link_name()

	var node: Node3D = godot_robot.links.get(link_name)
	_end_body = node as RigidBody3D
	if not _end_body:
		push_warning("BaseLinkLogger: link '%s' not found or not a RigidBody3D - not logging" % link_name)
		return

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
