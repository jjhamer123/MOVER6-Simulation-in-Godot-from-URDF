@tool
class_name GodotURDFImporter
extends EditorImportPlugin

func _get_importer_name() -> String:
	return "godot_urdf"
	
func _get_visible_name() -> String:
	return "Godot URDF"
	
func _get_recognized_extensions() -> PackedStringArray:
	return ["urdf"]

func _get_save_extension() -> String:
	return "tscn"
	
func _get_import_options(_path: String, _preset_index: int) -> Array[Dictionary]:
	return []
	
func _get_import_order() -> int:
	return 0
	
func _get_resource_type() -> String:
	return "PackedScene"
	
func _get_preset_count() -> int:
	return 1
	
func _get_preset_name(_preset_index: int) -> String:
	return "Default preset"
	
func _get_option_visibility(_path: String, _option_name: StringName, _options: Dictionary) -> bool:
	return false
	
func _get_priority() -> float:
	return 1.0
func _import(
		source_file: String, save_path: String, _options: Dictionary,
		_platform_variants: Array[String], _gen_files: Array[String]) -> Error:
	var scene := PackedScene.new()
	var urdf_parser := URDFXMLParser.new()

	var basename := source_file.get_basename()
	var source_dir_result := DirAccess.make_dir_recursive_absolute(basename)
	if source_dir_result != OK:
		push_error("Failed to create import directory: ", basename)
		return source_dir_result
	
	var options: Dictionary = URDFUtils.get_urdf_config(basename)
	URDFUtils.apply_defaults(options)

	var robot_node := urdf_parser.as_node3d(source_file, options, null, null)
	
	# Assign ownership recursively so PackedScene packs every joint node and metadata
	_set_owner_recursive(robot_node, robot_node)

	var pack_result := scene.pack(robot_node)
	if pack_result != OK:
		push_error("Failed to pack URDF node tree into scene.")
		return pack_result

	var saved_path := save_path + "." + _get_save_extension()
	var save_result := ResourceSaver.save(scene, saved_path)
	if save_result != OK:
		push_error("Failed to save imported .urdf as a scene.")
		return save_result

	return OK

func _set_owner_recursive(node: Node, scene_owner: Node) -> void:
	for child in node.get_children():
		if child != scene_owner:
			child.owner = scene_owner
		_set_owner_recursive(child, scene_owner)
