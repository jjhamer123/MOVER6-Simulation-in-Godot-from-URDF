@tool
extends Node3D
class_name ArmRig

@export_group("URDF")
@export_file("*.urdf", "*.xml") var urdf_file_path: String
@export_dir var package_folder: String = ""
@export_tool_button("Load robot") var _load_button = load_robot

@export_group("Transform")
@export var robot_position: Vector3 = Vector3.ZERO
@export var robot_rotation_deg: Vector3 = Vector3.ZERO
## 0 = use the URDF's own config / godot_urdf's default (1mm -> 1 unit)
@export var robot_scale: float = 0.0

@export_group("Downstream nodes")
@export var joint_controller: RobotJointController
@export var base_link_logger: BaseLinkLogger
@export var ros_bridge: RosArmBridge

var godot_robot: GodotRobot

func _ready() -> void:
	if Engine.is_editor_hint():
		return
	if not godot_robot:
		for child in get_children():
			if child is GodotRobot:
				godot_robot = child
				break
	if godot_robot:
		_wire_after_physics_settle()
	elif not urdf_file_path.is_empty():
		load_robot()

func load_robot() -> void:
	for child in get_children():
		if child is GodotRobot:
			child.free()
	godot_robot = null

	if urdf_file_path.is_empty():
		push_error("ArmRig: no URDF file selected")
		return

	var resolved_path := _resolve_path(urdf_file_path)
	if resolved_path.is_empty():
		push_error("ArmRig: failed to resolve path for '%s'" % urdf_file_path)
		return

	var parser := URDFXMLParser.new()
	var basename := resolved_path.get_basename()
	var options: Dictionary = URDFUtils.get_urdf_config(basename)

	if robot_scale > 0.0:
		options["scale"] = robot_scale
	if package_folder != "":
		options["package_folder"] = package_folder
	URDFUtils.apply_defaults(options)
	print("URDF options: ", options)
	
	var scene_root := get_tree().edited_scene_root if Engine.is_editor_hint() \
		else get_tree().current_scene

	godot_robot = parser.as_node3d(resolved_path, options, self, scene_root)
	if not godot_robot:
		push_error("ArmRig: failed to load '%s'" % resolved_path)
		return

	godot_robot.position = robot_position
	godot_robot.rotation = robot_rotation_deg / 180.0 * PI

	print("ArmRig: loaded '%s' from %s" % [godot_robot.name, resolved_path])
	
	if Engine.is_editor_hint():
		fix_meshes(godot_robot)
	else:
		_wire_after_physics_settle()

func _resolve_path(path: String) -> String:
	if path.begins_with("uid://"):
		var id := ResourceUID.text_to_id(path)
		if id != ResourceUID.INVALID_ID and ResourceUID.has_id(id):
			return ResourceUID.get_id_path(id)
		else:
			push_error("ArmRig: Unregistered or invalid UID -> " + path)
			return ""
	return path

func _wire_after_physics_settle() -> void:
	await get_tree().physics_frame
	if godot_robot:
		print("Fixing Meshes")
		fix_meshes(godot_robot)
	_wire_downstream()

func _wire_downstream() -> void:
	if godot_robot:
		godot_robot.rebuild_caches()
	if joint_controller:
		joint_controller.godot_robot = godot_robot
		joint_controller.reinitialize()
	if base_link_logger:
		base_link_logger.godot_robot = godot_robot
		base_link_logger.reinitialize()
	if ros_bridge and joint_controller:
		ros_bridge.robot_controller = joint_controller
		
# --- URDF XML Parsing & Visual Mesh Rotation ---

func get_compound_links_from_urdf(path: String) -> Array[String]:
	var compound_links: Array[String] = []
	var parser := XMLParser.new()
	if parser.open(path) != OK:
		push_error("ArmRig: Failed to open URDF at path: " + path)
		return compound_links

	var current_link := ""
	var in_mesh_element := false

	while parser.read() == OK:
		var node_type := parser.get_node_type()

		if node_type == XMLParser.NODE_ELEMENT:
			var tag_name := parser.get_node_name().to_lower()

			if tag_name == "link":
				current_link = parser.get_named_attribute_value_safe("name") if parser.has_attribute("name") else ""
			elif tag_name == "joint":
				current_link = ""
				in_mesh_element = false
			elif tag_name == "visual" or tag_name == "collision":
				in_mesh_element = true
			elif tag_name == "origin" and current_link != "" and in_mesh_element:
				if parser.has_attribute("rpy"):
					var rpy_str := parser.get_named_attribute_value_safe("rpy")
					var angles := rpy_str.split_floats(" ")

					var non_zero_count := 0
					for val in angles:
						if abs(val) > 0.001:
							non_zero_count += 1

					if non_zero_count > 1 and not current_link in compound_links:
						compound_links.append(current_link)

		elif node_type == XMLParser.NODE_ELEMENT_END:
			var tag_name := parser.get_node_name().to_lower()
			if tag_name == "link":
				current_link = ""
				in_mesh_element = false
			elif tag_name == "visual" or tag_name == "collision":
				in_mesh_element = false

	return compound_links

func fix_meshes(robot_node: Node3D) -> void:
	if urdf_file_path.is_empty():
		return
