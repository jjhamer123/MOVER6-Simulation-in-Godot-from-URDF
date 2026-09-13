@tool
class_name URDFRobot
extends Resource

@export var name: String
@export var links: Array = []
@export var joints: Array = []
@export var materials: Dictionary = {}

func get_child_joints(link_name: String) -> Array:
	var children: Array = []
	for joint in joints:
		if joint.parent == link_name:
			children.append(joint)
	return children
	
func get_link(link_name: String) -> Object:
	for link in links:
		if link.name == link_name:
			return link
	return null

func get_root_links() -> Array[String]:
	var roots: Array[String] = []
	for link in links:
		var has_parent = false
		for joint in joints:
			if joint.child == link.name:
				has_parent = true
				break
		if not has_parent:
			roots.append(link.name)
	return roots
