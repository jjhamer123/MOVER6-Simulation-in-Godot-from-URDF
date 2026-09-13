class_name URDFXMLParser
extends XMLParser

var resource_cache: Dictionary[String, Resource] = {}

func as_node3d(
		source_path: String,
		options: Dictionary,
		parent_node: Node3D,
		owner_node: Node3D) -> GodotRobot:
	var start_time = Time.get_ticks_msec()
	
	var resolved_path := _resolve_path(source_path)
	if resolved_path.is_empty():
		push_error("URDFXMLParser: Failed to resolve path -> " + source_path)
		return null

	print("parsing " + resolved_path)
	var robot: Resource = parse(resolved_path)
	if not robot:
		push_error("URDFXMLParser: Failed to build URDFRobot data")
		return null

	var robot_node = GodotRobot.new()
	robot_node.init_data(
		robot, parent_node, owner_node, options, resolved_path)

	var now = Time.get_ticks_msec()
	var elapsed = (now - start_time) / 1000.0
	print("Done generating robot, took:", elapsed)
	return robot_node

func _resolve_path(path: String) -> String:
	if path.begins_with("uid://"):
		var id := ResourceUID.text_to_id(path)
		if id != ResourceUID.INVALID_ID and ResourceUID.has_id(id):
			return ResourceUID.get_id_path(id)
		else:
			push_error("URDFXMLParser: Invalid or unregistered UID -> " + path)
			return ""
	return path

func parse(source_path: String) -> URDFRobot:
	var parser = XMLParser.new()
	var err = parser.open(source_path)
	if err != OK:
		push_error("Failed to open URDF file: " + source_path)
		return null

	var robot = URDFRobot.new()
	while parser.read() == OK:
		var node_type = parser.get_node_type()
		if node_type == XMLParser.NODE_TEXT:
			continue
		
		if node_type == XMLParser.NODE_ELEMENT:
			if parser.get_node_name() == "robot":
				robot.name = parser.get_named_attribute_value_safe("name")
				print("robot name: ", robot.name)
				parse_robot_children(parser, robot)
	return robot

func parse_robot_children(parser: XMLParser, robot: Object) -> void:
	while parser.read() == OK:
		var node_type = parser.get_node_type()
		if node_type == XMLParser.NODE_TEXT:
			continue
		var node_name = parser.get_node_name()
		
		if node_type == XMLParser.NODE_ELEMENT:
			match node_name:
				"link":
					robot.links.append(get_urdf_link(parser))
				"joint":
					robot.joints.append(get_urdf_joint(parser))
				"material":
					var material_name = parser.get_named_attribute_value_safe("name")
					robot.materials[material_name] = parse_material_color(parser)
				"transmission", "gazebo", "ros2_control":
					parser.skip_section()
				_:
					push_error("Unsupported tag in robot: ", node_name)
					parser.skip_section()

		elif node_type == XMLParser.NODE_ELEMENT_END:
			if node_name == "robot":
				return

func get_urdf_joint(parser: XMLParser) -> URDFJoint:
	var joint = URDFJoint.new()
	joint.name = parser.get_named_attribute_value_safe("name")
	joint.axis = Vector3(1, 0, 0)
	if parser.is_empty():
		return joint
	joint.type = parser.get_named_attribute_value_safe("type")

	while parser.read() == OK:
		var node_type = parser.get_node_type()
		if node_type == XMLParser.NODE_TEXT:
			continue

		var node_name = parser.get_node_name()
		if node_type == XMLParser.NODE_ELEMENT:
			match node_name:
				"parent":
					joint.parent = parser.get_named_attribute_value_safe("link")
				"child":
					joint.child = parser.get_named_attribute_value_safe("link")
				"axis":
					var parsed_axis := parse_xyz(parser)
					joint.axis = parsed_axis.normalized() if parsed_axis != Vector3.ZERO else Vector3(1, 0, 0)
				"origin":
					joint.origin_xyz = parse_xyz(parser)
					joint.origin_rpy = parse_rpy(parser)
				"limit":
					parse_joint_limit(parser, joint)
				"dynamics":
					parse_joint_dynamics(parser, joint)
				"joint_properties", "mimic", "safety_controller", "calibration":
					parser.skip_section()
				_:
					push_error("Unsupported tag in joint: ", node_name)
					parser.skip_section()
		elif node_type == XMLParser.NODE_ELEMENT_END:
			if node_name == "joint":
				return joint

	return joint

func get_urdf_link(parser: XMLParser) -> URDFLink:
	var link: URDFLink = URDFLink.new()
	link.name = parser.get_named_attribute_value_safe("name")
	if parser.is_empty():
		return link

	while parser.read() == OK:
		var node_type = parser.get_node_type()
		if node_type == XMLParser.NODE_TEXT:
			continue
		var node_name = parser.get_node_name()
		if node_type == XMLParser.NODE_ELEMENT:
			match node_name:
				"visual":
					link.visuals.append(get_link_visual(parser))
				"collision":
					link.colliders.append(get_link_collider(parser))
				"inertial":
					link.inertial = get_link_inertial(parser)
				"gazebo":
					parser.skip_section()
				_:
					parser.skip_section()
					push_error("Unsupported Tag in Link: ", node_name)
		elif node_type == XMLParser.NODE_ELEMENT_END:
			if node_name == "link":
				return link
	return link

func get_link_inertial(parser: XMLParser) -> URDFInertial:
	var inertial = URDFInertial.new()
	if parser.is_empty():
		return inertial
	while parser.read() == OK:
		var node_type = parser.get_node_type()
		if node_type == XMLParser.NODE_TEXT:
			continue
		var node_name = parser.get_node_name() 
		if node_type == XMLParser.NODE_ELEMENT:
			match node_name:
				"mass":
					var mass = parser.get_named_attribute_value_safe("value")
					if mass.is_valid_float():
						inertial.mass = float(mass)
				"origin":
					inertial.origin_xyz = parse_xyz(parser)
					inertial.origin_rpy = parse_rpy(parser)
				"inertia":
					inertial.inertia_tensor = parse_inertia(parser)
				_:
					push_error("Unsupported tag for inertial: ", node_name)
					parser.skip_section()
		elif node_type == XMLParser.NODE_ELEMENT_END:
			if node_name == "inertial":
				return inertial
	return inertial

func parse_inertia(parser: XMLParser) -> Dictionary:
	var inertia = {}
	for idx in range(parser.get_attribute_count()):
		var attr_name = parser.get_attribute_name(idx)
		var attr_value = parser.get_attribute_value(idx)
		if attr_value.is_valid_float():
			inertia[attr_name] = float(attr_value)
	return inertia

func get_link_collider(parser: XMLParser) -> URDFCollider:
	var collider = URDFCollider.new()

	if parser.is_empty():
		return collider

	while parser.read() == OK:
		var node_type = parser.get_node_type()
		if node_type == XMLParser.NODE_TEXT:
			continue
		var node_name = parser.get_node_name() 
		if node_type == XMLParser.NODE_ELEMENT:
			match node_name:
				"origin":
					collider.origin_xyz = parse_xyz(parser)
					collider.origin_rpy = parse_rpy(parser)
				"geometry":
					parse_geometry(parser, collider, false)
				_:
					push_error("Unsupported collider for Link: ", node_name)
					parser.skip_section()
		elif node_type == XMLParser.NODE_ELEMENT_END:
			if node_name == "collision":
				return collider
	return collider

func get_link_visual(parser: XMLParser) -> URDFVisual:
	var visual = URDFVisual.new()

	if parser.is_empty():
		return visual

	while parser.read() == OK:
		var node_type = parser.get_node_type()
		if node_type == XMLParser.NODE_TEXT:
			continue
		var node_name = parser.get_node_name()

		if node_type == XMLParser.NODE_ELEMENT:
			match node_name:
				"origin":
					visual.origin_xyz = parse_xyz(parser)
					visual.origin_rpy = parse_rpy(parser)
				"geometry":
					parse_geometry(parser, visual, true)
				"material":
					visual.material_name = parser.get_named_attribute_value_safe("name")
					visual.material_color = parse_material_color(parser)
				_:
					push_error("Unsupported node for Visual link: ", node_name)
					parser.skip_section()
		elif node_type == XMLParser.NODE_ELEMENT_END:
			if node_name == "visual":
				return visual
	return visual

func parse_xyz(parser: XMLParser) -> Vector3:
	return URDFUtils.parse_xyz(
		parser.get_named_attribute_value_safe("xyz"))

func parse_rpy(parser: XMLParser) -> Vector3:
	return URDFUtils.parse_rpy(
		parser.get_named_attribute_value_safe("rpy"))

func _get_named_float(parser: XMLParser, name: String) -> float:
	var value = parser.get_named_attribute_value_safe(name)
	return 0.0 if value.is_empty() else float(value)

func parse_joint_limit(parser: XMLParser, joint: Object) -> void:
	joint.limit = URDFJoint.URDFLimit.new()
	joint.limit.lower = _get_named_float(parser, "lower")
	joint.limit.upper = _get_named_float(parser, "upper")
	joint.limit.effort = _get_named_float(parser, "effort")
	joint.limit.velocity = _get_named_float(parser, "velocity")

func parse_joint_dynamics(parser: XMLParser, joint: Object) -> void:
	joint.dynamics = URDFJoint.URDFDynamics.new()
	joint.dynamics.damping = _get_named_float(parser, "damping")
	joint.dynamics.friction = _get_named_float(parser, "friction")

func parse_geometry(
		parser: XMLParser,
		target_object: Object,
		is_visual: bool) -> void:
	if parser.is_empty():
		return

	while parser.read() == OK:
		var node_type = parser.get_node_type()
		if node_type == XMLParser.NODE_TEXT:
			continue

		if node_type == XMLParser.NODE_ELEMENT:
			var node_name = parser.get_node_name()
			match node_name:
				"box":
					if is_visual:
						target_object.type = URDFVisual.Type.BOX
					else:
						target_object.type = URDFCollider.Type.BOX

					var size_split = parser.get_named_attribute_value_safe("size").split(" ", false)
					if size_split.size() >= 3:
						target_object.size = Vector3(
							float(size_split[0]),
							float(size_split[1]),
							float(size_split[2])
						)

				"cylinder":
					if is_visual:
						target_object.type = URDFVisual.Type.CYLINDER
					else:
						target_object.type = URDFCollider.Type.CYLINDER

					target_object.length = float(parser.get_named_attribute_value_safe("length"))
					target_object.radius = float(parser.get_named_attribute_value_safe("radius"))

				"sphere":
					if is_visual:
						target_object.type = URDFVisual.Type.SPHERE
					else:
						target_object.type = URDFCollider.Type.SPHERE

					target_object.radius = float(parser.get_named_attribute_value_safe("radius"))

				"mesh":
					if is_visual:
						target_object.type = URDFVisual.Type.MESH
					else:
						target_object.type = URDFCollider.Type.MESH

					target_object.mesh_path = parser.get_named_attribute_value_safe("filename")
					
					var scale_str = parser.get_named_attribute_value_safe("scale")
					if not scale_str.is_empty():
						var scale_split = scale_str.split(" ", false)
						if scale_split.size() >= 3:
							var parsed_scale := Vector3(
								float(scale_split[0]),
								float(scale_split[1]),
								float(scale_split[2])
							)
							if "mesh_scale" in target_object:
								target_object.mesh_scale = parsed_scale
							elif "scale" in target_object:
								target_object.scale = parsed_scale

				_:
					push_error("Unsupported geometry for visual in link properties: ", node_name)
					parser.skip_section()

		elif node_type == XMLParser.NODE_ELEMENT_END:
			if parser.get_node_name() == "geometry":
				return

func parse_material_color(parser: XMLParser) -> Vector4:
	var material_color = Vector4.ZERO
	if parser.is_empty():
		return material_color

	while parser.read() == OK:
		var node_type = parser.get_node_type()
		if node_type == XMLParser.NODE_TEXT:
			continue
		if node_type == XMLParser.NODE_ELEMENT and \
				parser.get_node_name() == "color":
			var rgba_str = parser.get_named_attribute_value_safe("rgba")
			if rgba_str and !rgba_str.is_empty():
				var color_split = rgba_str.split(" ", false)
				if color_split.size() >= 4:
					material_color = Vector4(
						float(color_split[0]),
						float(color_split[1]),
						float(color_split[2]),
						float(color_split[3]))
				else:
					push_warning("Invalid color: ", rgba_str)
		elif node_type == XMLParser.NODE_ELEMENT_END:
			if parser.get_node_name() == "material":
				return material_color
	return material_color
