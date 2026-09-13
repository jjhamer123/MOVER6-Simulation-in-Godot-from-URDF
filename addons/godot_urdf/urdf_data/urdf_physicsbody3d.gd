class_name URDFPhysicsBody3D

static func update_link(
		body: PhysicsBody3D,
		link: URDFLink,
		robot_node: GodotRobot,
		owner_node: Node3D,
		options: Dictionary,
		source_path: String) -> void:
	body.name = link.name
	
	# Only parent to robot_node if the body hasn't already been attached to a joint pivot node
	if body.get_parent() == null:
		robot_node.add_child(body)
		if owner_node:
			body.owner = owner_node

	if link.inertial:
		body.mass = link.inertial.mass
	
	for col_data in link.colliders:
		var gen = URDFGeometryFactory.get_collision_callable(int(col_data.type))
		if gen:
			gen.call(body, owner_node, col_data, options, source_path)
		else:
			push_error("Unknown type ", col_data.type)

	for visual in link.visuals:
		var material = StandardMaterial3D.new()
		var c = visual.material_color
		if c != Vector4.ZERO:
			material.albedo_color = Color(c.x, c.y, c.z, c.w)
		elif robot_node.urdf and visual.material_name in robot_node.urdf.materials:
			c = robot_node.urdf.materials[visual.material_name]
			material.albedo_color = Color(c.x, c.y, c.z, c.w)

		var gen = URDFGeometryFactory.get_visual_callable(int(visual.type))
		if gen:
			gen.call(
				body, owner_node, visual, options, source_path,
				null if c == Vector4.ZERO else material)
		else:
			push_error("Unknown type ", visual.type)
