@tool
extends Node3D

# Change this to match your specific metadata key or Godot Group name
const TAG_NAME := "is_stl"

func _ready() -> void:
	fix_tagged_meshes(self)

func fix_tagged_meshes(node: Node) -> void:
	if node is Node3D:
		_process_node(node)

	for child in node.get_children():
		fix_tagged_meshes(child)

func _process_node(node_3d: Node3D) -> void:
	print_debug("Checking node rotation start:")
	# Avoid double-rotating if already processed
	if node_3d.has_meta("_z_up_corrected"):
		return

	var name_lower := node_3d.name.to_lower()
	var ends_with_mesh := name_lower.ends_with("mesh")
	
	# Checks if the node has the metadata tag OR belongs to a Godot group
	var has_tag := node_3d.has_meta(TAG_NAME) or node_3d.is_in_group(TAG_NAME)

	if has_tag and ends_with_mesh:
		# Applies 90-degree rotation along local X-axis (use -90.0 if direction is inverted)
		node_3d.rotate_object_local(Vector3.RIGHT, deg_to_rad(90.0))
		print_debug("Checking node rotation: "+node_3d.name)
		node_3d.set_meta("_z_up_corrected", true)
