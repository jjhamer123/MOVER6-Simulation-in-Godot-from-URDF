class_name URDFGeometryFactory

static var resource_cache: Dictionary = {}

static func _clean_path(
		package_path: String,
		options: Dictionary,
		source_path: String) -> String:
	var clean_path = package_path.replace("package://", "")
	if options.has("package_folder"):
		return options["package_folder"].path_join(clean_path)
	# Fallback: try to find it relative to the URDF file
	return source_path.get_base_dir().path_join(clean_path)

static func load_resource(
		path: String, opts: Dictionary, source_path: String) -> Resource:
	path = _clean_path(path, opts, source_path)
	if resource_cache.has(path): return resource_cache[path]
	var res = load(path)
	resource_cache[path] = res
	return res

static func get_visual_callable(type: int) -> Callable:
	match type:
		URDFVisual.Type.BOX:
			return create_box_visual
		URDFVisual.Type.CYLINDER:
			return create_cylinder_visual
		URDFVisual.Type.SPHERE:
			return create_sphere_visual
		URDFVisual.Type.MESH:
			return create_mesh_resource_visual
	return Callable()

static func get_collision_callable(type: int) -> Callable:
	match type:
		URDFCollider.Type.BOX:
			return create_box_collision
		URDFCollider.Type.CYLINDER:
			return create_cylinder_collision
		URDFCollider.Type.SPHERE:
			return create_sphere_collision
		URDFCollider.Type.MESH:
			return create_mesh_resource_collision
	return Callable()

# --- Orientation Helper ---

static func _get_mesh_local_fix(mesh_path: String, opts: Dictionary) -> Transform3D:
	var local_fix := Transform3D.IDENTITY
	if opts.get('rotate_x', null) != null:
		return local_fix.rotated(Vector3.RIGHT, opts['rotate_x'])
	
	var ext = mesh_path.get_extension().to_lower()
	if ext == "stl" or ext == "obj" or mesh_path.contains("upperarm") or mesh_path.contains("forearm"):
		local_fix = local_fix.rotated(Vector3.RIGHT, -PI / 2.0)
		
	return local_fix

# --- Visual Generators ---

static func create_box_visual(
		parent: Node3D, owner: Node, data: URDFVisual,
		_opts: Dictionary, _path: String, material: BaseMaterial3D):
	var mesh_inst = MeshInstance3D.new()
	mesh_inst.mesh = BoxMesh.new()
	mesh_inst.mesh.size = data.size
	_finalize(mesh_inst, parent, owner, material, data.origin_xyz, data.origin_rpy)

static func create_cylinder_visual(
		parent: Node3D, owner: Node, data: URDFVisual,
		_opts: Dictionary, _path: String, material: BaseMaterial3D):
	var mesh_inst = MeshInstance3D.new()
	var cm = CylinderMesh.new()
	cm.height = data.length
	cm.top_radius = data.radius
	cm.bottom_radius = data.radius
	mesh_inst.mesh = cm
	
	var local_fix = Transform3D.IDENTITY.rotated(Vector3.RIGHT, PI / 2.0)
	_finalize_with_local_fix(mesh_inst, parent, owner, material, data.origin_xyz, data.origin_rpy, local_fix)

static func create_sphere_visual(
		parent: Node3D, owner: Node, data: URDFVisual,
		_opts: Dictionary, _path: String, material: BaseMaterial3D):
	var mesh_inst = MeshInstance3D.new()
	var sphere = SphereMesh.new()
	sphere.radius = data.radius
	sphere.height = data.radius * 2.0
	mesh_inst.mesh = sphere
	_finalize(mesh_inst, parent, owner, material, data.origin_xyz, data.origin_rpy)

static func create_mesh_resource_visual(
		parent: Node3D, owner: Node, data: URDFVisual,
		opts: Dictionary, source_path: String, material: BaseMaterial3D):
	var resource = load_resource(data.mesh_path, opts, source_path)
	var instance: Node3D
	if resource is PackedScene:
		instance = resource.instantiate()
	elif resource is Mesh:
		var mesh_inst = MeshInstance3D.new()
		mesh_inst.mesh = resource
		instance = mesh_inst
	else:
		push_error("Error loading " + data.mesh_path + " - Unknown Resource type:" + type_string(typeof(resource)))
		return

	var local_fix = _get_mesh_local_fix(data.mesh_path, opts)
	_finalize_with_local_fix(instance, parent, owner, material, data.origin_xyz, data.origin_rpy, local_fix)

	var global_scale = opts.get("scale", 1.0)
	instance.scale = data.mesh_scale * global_scale
	# --- Collision Generators ---

static func create_box_collision(
		parent: Node3D, owner: Node, data: URDFCollider,
		_opts: Dictionary, _path: String):
	var coll = CollisionShape3D.new()
	coll.shape = BoxShape3D.new()
	coll.shape.size = data.size
	_finalize(coll, parent, owner, null, data.origin_xyz, data.origin_rpy)

static func create_cylinder_collision(
		parent: Node3D, owner: Node, data: URDFCollider,
		_opts: Dictionary, _path: String):
	var coll = CollisionShape3D.new()
	var shape = CylinderShape3D.new()
	shape.height = data.length
	shape.radius = data.radius
	coll.shape = shape
	
	var local_fix = Transform3D.IDENTITY.rotated(Vector3.RIGHT, PI / 2.0)
	_finalize_with_local_fix(coll, parent, owner, null, data.origin_xyz, data.origin_rpy, local_fix)

static func create_sphere_collision(
		parent: Node3D, owner: Node, data: URDFCollider,
		_opts: Dictionary, _path: String):
	var coll = CollisionShape3D.new()
	var shape = SphereShape3D.new()
	shape.radius = data.radius
	coll.shape = shape
	_finalize(coll, parent, owner, null, data.origin_xyz, data.origin_rpy)

static func create_mesh_resource_collision(
		parent: Node3D, owner: Node, data: URDFCollider,
		opts: Dictionary, source_path: String):
	opts["mesh_path"] = data.mesh_path
	opts["mesh_scale"] = data.mesh_scale
	var resource = load_resource(data.mesh_path, opts, source_path)
	var urdf_transform = URDFUtils.xyz_rpy_to_transform3d(data.origin_xyz, data.origin_rpy)
	var ext = data.mesh_path.get_extension().to_lower()

	if resource is Mesh:
		_create_col_shape_from_mesh(resource, urdf_transform, parent, owner, opts, ext)
	elif resource is PackedScene:
		var temp_scene = resource.instantiate()
		_recursive_collision_gen(temp_scene, urdf_transform, parent, owner, opts, ext)
		temp_scene.queue_free()

static func _recursive_collision_gen(
		node: Node, base_transform: Transform3D, 
		parent: Node3D, owner: Node,
		opts: Dictionary, ext: String = ""):
	if node is MeshInstance3D:
		var final_transform = base_transform * node.transform
		_create_col_shape_from_mesh(node.mesh, final_transform, parent, owner, opts, ext)
	
	for child in node.get_children():
		_recursive_collision_gen(child, base_transform, parent, owner, opts, ext)

static func _create_col_shape_from_mesh(
		mesh: Mesh, tr: Transform3D, 
		parent: Node3D, owner: Node,
		opts: Dictionary, ext: String = ""):
	var shape = mesh.create_convex_shape(true, true)
	if shape:
		var coll = CollisionShape3D.new()
		coll.shape = shape
		coll.name = parent.name + "_collision"
		parent.add_child(coll)
		coll.owner = owner
		
		var mesh_path = opts.get("mesh_path", "")
		var local_fix = _get_mesh_local_fix(mesh_path, opts)

		coll.transform = tr * local_fix
		
		var mesh_scale: Vector3 = opts.get("mesh_scale", Vector3.ONE)
		var s = opts.get("scale", 1.0)
		coll.scale = mesh_scale * s

# --- Finalize Helpers ---

static func _finalize(
		node: Node3D, parent: Node, owner: Node, 
		material: BaseMaterial3D,
		xyz: Vector3, rpy: Vector3, num: int = 0):
	_finalize_with_local_fix(node, parent, owner, material, xyz, rpy, Transform3D.IDENTITY, num)

static func _finalize_with_local_fix(
		node: Node3D, parent: Node, owner: Node, 
		material: BaseMaterial3D,
		xyz: Vector3, rpy: Vector3,
		local_fix: Transform3D, num: int = 0):
	parent.add_child(node)
	if node is CollisionShape3D:
		node.name = parent.name + "_collision"
	else:
		node.name = parent.name + "_mesh"
		if material:
			node.material_override = material
	if num > 0:
		node.name += "_" + str(num)
	node.owner = owner
	
	var urdf_tr = URDFUtils.xyz_rpy_to_transform3d(xyz, rpy)
	node.transform = urdf_tr * local_fix
