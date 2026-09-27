extends Node
class_name BaseLinkLogger
 
@export var godot_robot: GodotRobot
@export var log_interval_sec: float = 1
@export var write_to_file: bool = true
#optional, force which link track (URDF link name). Leave blank to
#auto-pick the deepest leaf link (end-effector for arm).
#Set explicitly for grippers/branching arms where auto-pick might grab
#the wrong one
@export var end_link_name: String = ""
 
var _end_body: RigidBody3D
var _timer: float = 0.0
var _start_transform: Transform3D
 
func _ready() -> void:
	reinitialize()


func reinitialize() -> void:
	if not godot_robot:
		push_warning("BaseLinkLogger: no godot_robot assigned")
		return

	var link_name := end_link_name
	if link_name.is_empty():
		link_name = _find_deepest_real_leaf_link()

	var node: Node3D = godot_robot.links.get(link_name)
	_end_body = node as RigidBody3D
	if not _end_body:
		push_warning("BaseLinkLogger: link '%s' not found or not a RigidBody3D - not logging" % link_name)
		return

	_start_transform = _end_body.global_transform

#find the end link

func _find_deepest_real_leaf_link() -> String:
	if not godot_robot or not godot_robot.urdf:
		return ""

	var real_links: Dictionary = godot_robot.links
	var parent_of: Dictionary = {}
	var has_real_child: Dictionary = {}
	for joint in godot_robot.urdf.joints:
		if real_links.has(joint.parent) and real_links.has(joint.child):
			parent_of[joint.child] = joint.parent
			has_real_child[joint.parent] = true

	var leaves: Array = []
	for link_name in real_links.keys():
		if not has_real_child.has(link_name):
			leaves.append(link_name)

	var depth_cache: Dictionary = {}
	var deepest_name := ""
	var deepest_depth := -1
	for leaf in leaves:
		var depth := _real_link_depth(leaf, parent_of, depth_cache)
		if depth > deepest_depth:
			deepest_depth = depth
			deepest_name = leaf
	return deepest_name

func _real_link_depth(link_name: String, parent_of: Dictionary, cache: Dictionary) -> int:
	if cache.has(link_name):
		return cache[link_name]
	if not parent_of.has(link_name):
		cache[link_name] = 0
		return 0
	var depth: int = 1 + _real_link_depth(parent_of[link_name], parent_of, cache)
	cache[link_name] = depth
	return depth

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
