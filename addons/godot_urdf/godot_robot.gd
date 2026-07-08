@tool
class_name GodotRobot
extends Node3D

var _transform_cache: Dictionary[String, Transform3D] = {}
var _joint_defs: Dictionary[String, Dictionary] = {}

var links: Dictionary[String, Node3D] = {}

@export var urdf: URDFRobot
@export var pin_base_link: bool = true

func add_joint(joint: URDFJoint, local_transform: Transform3D):
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
		robot: URDFRobot,
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
			# neither visual elements nor collider, skip
			print("URDF DEBUG: link '", link.name, "' has no colliders and no visuals, skipped entirely")
			continue

		link_node.update_link(
			link, self, owner, options, source_path)
		links[link.name] = link_node
		print("URDF DEBUG: link '", link.name, "' -> ", link_node.get_class())

				
	for joint in robot.joints:
		var child_node: Node3D = links.get(joint.child)

		# Set center position and store in cache so we can
		# calculate the global positions later.
		var local_transform: Transform3D = URDFUtils.xyz_rpy_to_transform3d(
				joint.origin_xyz, joint.origin_rpy)

		if child_node:
			child_node.name = joint.name
		self.add_joint(joint, local_transform)
		self.create_godot_joint(joint, owner)

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

	for link_name in links.keys():
		var global_rel_transform = self.get_rel_transform(link_name)
		var link_node = links[link_name]
		link_node.transform = global_rel_transform

func create_godot_joint(joint: URDFJoint, owner: Node3D):
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
	godot_joint.update_joint(self, owner, joint)

	godot_joint.node_a = godot_joint.get_path_to(collision_node_a)
	godot_joint.node_b = godot_joint.get_path_to(collision_node_b)
