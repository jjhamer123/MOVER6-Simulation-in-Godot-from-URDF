@tool
class_name GodotRobot
extends Node3D

var _transform_cache: Dictionary[String, Transform3D] = {}
var _joint_defs: Dictionary[String, Dictionary] = {}

var links: Dictionary[String, Node3D] = {}

@export var urdf: Resource
@export var pin_base_link: bool = true


func rebuild_caches() -> void:
	links.clear()
	_joint_defs.clear()
	_transform_cache.clear()

	if not urdf:
		return

	var child_link_to_joint_name: Dictionary[String, String] = {}
	for joint in urdf.joints:
		child_link_to_joint_name[joint.child] = joint.name

	for link in urdf.links:
		var node_name: String = child_link_to_joint_name.get(link.name, link.name)
		var node := get_node_or_null(node_name)
		if node:
			links[link.name] = node

	for joint in urdf.joints:
		var local_transform := _get_corrected_joint_transform(joint)
		_joint_defs[joint.child] = {
			"parent": joint.parent,
			"transform": local_transform
		}


func add_joint(joint: Object, local_transform: Transform3D) -> void:
	_joint_defs[joint.child] = {
		"parent": joint.parent,
		"transform": local_transform
	}


func get_rel_transform(link_name: String) -> Transform3D:
	if _transform_cache.has(link_name):
		return _transform_cache[link_name]
	
	if not _joint_defs.has(link_name):
		_transform_cache[link_name] = Transform3D.IDENTITY
		return Transform3D.IDENTITY
	
	var joint = _joint_defs[link_name]
	var parent_transform = get_rel_transform(joint.parent)
	var transform = parent_transform * joint.transform
	
	_transform_cache[link_name] = transform
	return transform


func init_data(
		robot: Resource,
		parent: Node3D,
		owner: Node3D,
		options: Dictionary,
		source_path: String) -> void:
	self.urdf = robot
	self.name = robot.name
	if parent:
		parent.add_child(self)
	if owner:
		self.owner = owner
	else:
		owner = self
		
	var child_link_names: Dictionary[String, bool] = {}
	for joint in robot.joints:
		child_link_names[joint.child] = true

	# 1. Create link nodes
	for link in robot.links:
		var link_node = null
		if link.colliders.size() > 0:
			var physics: bool = options.get("create_physics", true)
			var physics_body = URDFRigidBody3D.new() \
				 if physics else URDFStaticBody3D.new()
			link_node = physics_body
		elif link.visuals.size() > 0:
			link_node = URDFVisualNode.new()

		if not link_node:
			print("URDF DEBUG: link '", link.name, "' has no colliders and no visuals, skipped entirely")
			continue

		link_node.update_link(
			link, self, owner, options, source_path)
		links[link.name] = link_node
		print("URDF DEBUG: link '", link.name, "' -> ", link_node.get_class())

	# 2. Cache joint transforms with zero translation modifications
	for joint in robot.joints:
		var local_transform := _get_corrected_joint_transform(joint)
		self.add_joint(joint, local_transform)

	# 3. Position link nodes globally relative to kinematic tree
	for link_name in links.keys():
		var global_rel_transform = self.get_rel_transform(link_name)
		var link_node = links[link_name]
		link_node.transform = global_rel_transform

	# 4. Instantiate Godot physics joints at their calculated locations
	for joint in robot.joints:
		var child_node: Node3D = links.get(joint.child)
		if child_node:
			child_node.name = joint.name
		self.create_godot_joint(joint, owner)

	# 5. Pin base link
	if pin_base_link:
		var physically_connected: Dictionary[String, bool] = {}
		for joint in robot.joints:
			var parent_body := links.get(joint.parent) as RigidBody3D
			var child_body := links.get(joint.child) as RigidBody3D
			if parent_body and child_body:
				physically_connected[joint.child] = true

		for link_name in links.keys():
			var body := links[link_name] as RigidBody3D
			if body and not physically_connected.has(link_name):
				body.freeze_mode = RigidBody3D.FREEZE_MODE_STATIC
				body.freeze = true
				print("URDF DEBUG: pinning '", link_name, "' as static root")


func get_root_link_names() -> Array[String]:
	return urdf.get_root_links() if urdf and urdf.has_method("get_root_links") else []


func get_leaf_link_names() -> Array[String]:
	var parent_names: Dictionary[String, bool] = {}
	if urdf:
		for joint in urdf.joints:
			parent_names[joint.parent] = true
	var leaves: Array[String] = []
	for link_name in links.keys():
		if not parent_names.has(link_name):
			leaves.append(link_name)
	return leaves


func get_deepest_leaf_link_name() -> String:
	var depth_cache: Dictionary[String, int] = {}
	var deepest_name := ""
	var deepest_depth := -1
	for leaf in get_leaf_link_names():
		var depth := _get_link_depth(leaf, depth_cache)
		if depth > deepest_depth:
			deepest_depth = depth
			deepest_name = leaf
	return deepest_name


func _get_link_depth(link_name: String, cache: Dictionary[String, int]) -> int:
	if cache.has(link_name):
		return cache[link_name]
	if not _joint_defs.has(link_name):
		cache[link_name] = 0
		return 0
	var depth: int = 1 + _get_link_depth(_joint_defs[link_name].parent, cache)
	cache[link_name] = depth
	return depth


func create_godot_joint(joint: Object, owner: Node3D) -> void:
	var collision_node_a = links.get(joint.parent)
	var collision_node_b = links.get(joint.child)
	if !collision_node_a:
		print("URDF DEBUG: Can not find parent link '" + joint.parent + "' for joint '" + joint.name + "'")
		return
	if !collision_node_b:
		print("URDF DEBUG: Can not find child link '" + joint.child + "' for joint '" + joint.name + "'")
		return

	print("URDF DEBUG: creating joint '" + joint.name + "': " +
		joint.parent + " (" + collision_node_a.get_class() + ") -> " +
		joint.child + " (" + collision_node_b.get_class() + ")")
		
	var godot_joint: URDF6DOFJoint3D = URDF6DOFJoint3D.new()
	godot_joint.transform = get_rel_transform(joint.child)
	godot_joint.update_joint(self, owner, joint)

	godot_joint.node_a = godot_joint.get_path_to(collision_node_a)
	godot_joint.node_b = godot_joint.get_path_to(collision_node_b)


func _get_corrected_joint_transform(joint: Object) -> Transform3D:
	var local_basis := Basis.from_euler(joint.origin_rpy)  # default order (YXZ), matches parse_rpy's encoding
	return Transform3D(local_basis, joint.origin_xyz)
