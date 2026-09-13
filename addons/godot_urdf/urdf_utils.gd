class_name URDFUtils

static func apply_defaults(options: Dictionary):
	if "scale" not in options:
		options["scale"] = 1.0
	if "package_folder" not in options:
		options["package_folder"] = "res://urdf"
	if "create_physics" not in options:
		options["create_physics"] = true

static func _load_yaml(yaml_config: String, parser: YAMLParser):
	var options = null
	if FileAccess.file_exists(yaml_config):
		var yaml_file = FileAccess.open(yaml_config, FileAccess.READ)
		var yaml_text = yaml_file.get_as_text()
		options = parser.parse(yaml_text)
		yaml_file.close()
	return options

static func get_urdf_config(basename):
	var yaml_parser = YAMLParser.new()
	var options = _load_yaml(basename + ".yml", yaml_parser)
	if options:
		return options
	options = _load_yaml(basename + ".yaml", yaml_parser)
	if options:
		return options
	return {}

static func parse_xyz(xyz: String) -> Vector3:
	var xyz_split = xyz.split(" ", false)
	if xyz.is_empty() or xyz_split.size() < 3:
		push_error("not enough values for XYZ!")
		return Vector3(0, 0, 0)
	return Vector3(
		float(xyz_split[0]),
		float(xyz_split[2]),
		-float(xyz_split[1]))

static func parse_rpy(rpy: String) -> Vector3:
	if rpy.is_empty():
		return Vector3.ZERO
	var rpy_split = rpy.split(" ", false)
	if rpy_split.size() < 3:
		push_error("not enough values for RPY: " + rpy)
		return Vector3.ZERO
	
	var roll := float(rpy_split[0])
	var pitch := float(rpy_split[1])
	var yaw := float(rpy_split[2])

	# 1. Build rotation matrix in URDF Z-up space
	var urdf_basis := Basis.IDENTITY
	urdf_basis = urdf_basis.rotated(Vector3(1, 0, 0), roll)
	urdf_basis = urdf_basis.rotated(Vector3(0, 1, 0), pitch)
	urdf_basis = urdf_basis.rotated(Vector3(0, 0, 1), yaw)

	# 2. Convert rotation matrix to Godot Y-up space (-90 deg X axis remap)
	var axis_remap := Basis(Vector3(1, 0, 0), -PI / 2.0)
	var godot_basis := axis_remap * urdf_basis * axis_remap.inverse()

	# 3. Return converted Euler angles for Godot
	return godot_basis.get_euler()

static func xyz_rpy_to_transform3d(xyz: Vector3, rpy: Vector3) -> Transform3D:
	# xyz is mapped via parse_xyz, rpy is converted to Godot space via parse_rpy
	return Transform3D(Basis.from_euler(rpy), xyz)
# URDFUtils.gd

static func get_corrected_joint_data(
		joint_name: String,
		origin_xyz: Vector3,
		origin_rpy: Vector3,
		raw_axis: Vector3) -> Dictionary:

	# 1. Convert URDF origin RPY using extrinsic XYZ order
	var urdf_basis := Basis.from_euler(origin_rpy, EULER_ORDER_XYZ)

	# 2. Determine correction angle based on joint position in chain
	var correction_rpy := Vector3.ZERO
	if joint_name in ["base_link", "joint1", "joint2"]:
		# -90 degrees on local X-axis
		correction_rpy = Vector3(deg_to_rad(-90.0), 0.0, 0.0)
	else:
		# +90 degrees on local Y-axis
		correction_rpy = Vector3(0.0, deg_to_rad(90.0), 0.0)

	var correction_basis := Basis.from_euler(correction_rpy, EULER_ORDER_XYZ)

	# 3. Post-multiply to apply correction in local frame
	var final_basis := urdf_basis * correction_basis

	# 4. Rotate the raw URDF axis vector through the final orientation matrix
	var norm_axis := raw_axis.normalized() if raw_axis != Vector3.ZERO else Vector3(1, 0, 0)
	var final_axis := (final_basis * norm_axis).normalized()

	return {
		"transform": Transform3D(final_basis, origin_xyz),
		"axis": final_axis
	}
