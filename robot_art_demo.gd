extends Node3D
## Robot Art digital twin: loads the epicycle robot URDF when the scene starts,
## attaches RobotJointController (+ ros_2_arm.gd if the ROS 2 build is in use)
## and draws the pen-tip trace on the paper.
##
## Every layer's 500 mm support bar points the same way and runs in two
## frame-mounted tracks, so every motor body keeps facing the same way and
## joint k sets the ABSOLUTE direction of arm k:  pen = sum r_k (cos q_k, sin q_k).
##
## Joint names (3-layer URDF):
##   joint1..3   stepper motors (rad)                       - you drive these
##   clampB1..3  arm radius in metres (0.03 .. 0.17)         - you set these
##   pivot2..3   top bearings, = -joint of the layer above  - follow automatically
##   support1..3 support bar slide through the carriage (m) - follow automatically

@export_file("*.urdf") var urdf_file_path: String = "res://robot_art/robot_art_3_layers.urdf"

@export_group("Physics")
## true  = links posed exactly from the joint values (best for checking drawings).
## false = full Jolt physics (needed to see a free top pivot swing, inertia, etc.).
@export var kinematic_mode: bool = true
## Physics mode only: keeps the Jolt joint chain stiff (light bars carrying
## heavy motors sag otherwise).
@export var min_link_mass: float = 0.5

## Arm radius built into the Godot URDF's zero pose (xacro rest_radius:=0.07).
@export var rest_radius: float = 0.07

@export_group("Start pose")
## Arm radius of each layer in metres (clamped to 0.03 .. 0.17).
@export var radii: Array[float] = [0.10, 0.06, 0.04]

@export_group("Demo (when nothing is driving it over ROS)")
@export var demo_spin: bool = true
## Fourier frequency of each layer (n_k). Base speed w0 in rad/s.
## Motor k turns at n_k * w0 (the motor bodies never turn, so no subtraction).
@export var frequencies: Array[int] = [1, -2, 3]
@export var w0: float = 0.4

@export_group("ROS 2")
## Adds res://ros_2_arm.gd (JointJog in, /joint_states out) if the ROS 2 classes exist.
@export var use_ros: bool = true

@export_group("Pen trace")
@export var pen_down: bool = true
@export var trace_color: Color = Color(0.05, 0.05, 0.1)
@export var max_trace_points: int = 20000
## Pen tip offset from the 'pen' link, in Godot axes (URDF z = Godot y).
@export var pen_tip_offset: Vector3 = Vector3(0, -0.13, 0)
## World height of the paper surface (the trace is drawn here).
@export var paper_height: float = 0.0

var robot: GodotRobot
var controller: RobotJointController
var ros: Node
var _trace: PackedVector3Array = []
var _trace_mesh := ImmediateMesh.new()
var _trace_material := StandardMaterial3D.new()


func _ready() -> void:
	var options: Dictionary = URDFUtils.get_urdf_config(urdf_file_path.get_basename())
	if not options.has("package_folder"):
		options["package_folder"] = urdf_file_path.get_base_dir()  # meshes sit next to the URDF
	URDFUtils.apply_defaults(options)
	robot = URDFXMLParser.new().as_node3d(urdf_file_path, options, self, self)
	if robot == null:
		push_error("Robot Art: could not load " + urdf_file_path)
		return

	controller = RobotJointController.new()
	controller.name = "RobotJointController"
	controller.godot_robot = robot
	controller.min_link_mass = min_link_mass
	controller.kinematic_mode = kinematic_mode
	controller.disable_self_collision = true
	# The Godot URDFs are built with the arm at 0.07 m in their zero pose; this
	# offset makes every clampB value the true radius in metres, exactly like
	# the ROS URDF (epicycle_layers.urdf) that MATLAB uses.
	var offsets: Dictionary[String, float] = {}
	for j in robot.urdf.joints:
		if j.type == "prismatic" and j.name.begins_with("clampB"):
			offsets[j.name] = rest_radius
	controller.joint_offsets = offsets
	# Jolt's 6DOF motors turn the opposite way to the URDF axis; tell the
	# controller up front instead of letting it learn this after the first move.
	if not kinematic_mode:
		for j in robot.urdf.joints:
			if j.type in ["continuous", "revolute"] and j.mimic_joint == "" and j.limit and j.limit.effort > 0.0:
				controller.motor_direction_overrides[j.name] = -1.0
	add_child(controller)

	if use_ros and ClassDB.class_exists("JointJogController") and ResourceLoader.exists("res://ros_2_arm.gd"):
		ros = Node.new()
		ros.name = "ROS2Arm"
		ros.set_script(load("res://ros_2_arm.gd"))
		ros.set("robot_controller", controller)
		add_child(ros)
		print("Robot Art: ROS 2 bridge active (/JointJog in, /joint_states out)")

	# support bars: kinematic mode keeps each one centred between its tracks
	for j in robot.urdf.joints:
		if j.type == "prismatic" and j.name.begins_with("support"):
			var t = _track_centre(j.name)
			if t != null:
				controller.guided_slides[j.name] = t

	var trace_node := MeshInstance3D.new()
	trace_node.name = "PenTrace"
	trace_node.mesh = _trace_mesh
	_trace_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_trace_material.albedo_color = trace_color
	trace_node.material_override = _trace_material
	add_child(trace_node)

	# wait for the controller to finish its own setup
	for i in 2:
		await get_tree().physics_frame
	if not kinematic_mode:
		_add_physics_guides()
	await get_tree().physics_frame
	for k in radii.size():
		set_radius(k + 1, radii[k])
	if demo_spin and ros == null:
		start_demo()


## Arm radius of layer k (1-based), metres.
func set_radius(layer: int, radius_m: float) -> void:
	var j := _joint("clampB", layer)
	if j != "":
		controller.set_joint_angle(j, radius_m)


func get_radius(layer: int) -> float:
	var j := _joint("clampB", layer)
	return controller.get_joint_angle(j) if j != "" else 0.0


## Motor of layer k (1-based) at a speed in rad/s. Because the motor bodies never
## turn, this is also the absolute turning speed of arm k.
func set_motor_speed(layer: int, rad_s: float) -> void:
	var j := _joint("joint", layer)
	if j != "":
		controller.set_joint_velocity(j, rad_s * controller.get_motor_direction(j))


func start_demo() -> void:
	for k in frequencies.size():
		set_motor_speed(k + 1, frequencies[k] * w0)


## Centre point between a support bar's two tracks (global), or null.
func _track_centre(support_joint: String) -> Variant:
	var suffix := support_joint.substr("support".length())   # "2" or "" (single layer)
	var l: Node3D = robot.links.get("track%s_L" % suffix)
	var r: Node3D = robot.links.get("track%s_R" % suffix)
	if l == null or r == null:
		return null
	return (l.global_position + r.global_position) * 0.5


## Physics mode: hold every support bar like the real tracks do - it can move
## along the tracks, but not along its own length, up/down, or turn.
func _add_physics_guides() -> void:
	for j in robot.urdf.joints:
		if not (j.type == "prismatic" and j.name.begins_with("support")):
			continue
		var t = _track_centre(j.name)
		var slide := robot.get_node_or_null("joint_" + j.name) as Generic6DOFJoint3D
		var bar := robot.links.get(j.child) as RigidBody3D
		if t == null or slide == null or bar == null:
			continue
		var along: Vector3 = slide.global_transform.basis.z.normalized()   # bar direction
		# start with the bar centred between its tracks
		bar.global_position += along * (t - bar.global_position).dot(along)
		var anchor := StaticBody3D.new()
		anchor.name = "TrackFrame_" + j.name
		add_child(anchor)
		anchor.global_position = bar.global_position
		var guide := Generic6DOFJoint3D.new()
		guide.name = "TrackGuide_" + j.name
		add_child(guide)
		var x := along
		var y := Vector3.UP
		guide.global_transform = Transform3D(Basis(x, y, x.cross(y)), bar.global_position)
		guide.node_a = guide.get_path_to(anchor)
		guide.node_b = guide.get_path_to(bar)
		# Only lock what the tracks really add: no sliding along the bar (x) and no
		# turning about the vertical (y). Height and tilt are already set by the
		# chain above, and locking them twice makes Jolt bind.
		guide.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, false)
		guide.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, false)  # free along the tracks
		guide.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, false)
		guide.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, false)


func clear_trace() -> void:
	_trace.clear()


func get_pen_tip() -> Vector3:
	var pen: Node3D = robot.links.get("pen") if robot else null
	return pen.global_transform * pen_tip_offset if pen else Vector3.ZERO


func _joint(prefix: String, layer: int) -> String:
	var names := controller.get_joint_names()
	if names.has(prefix + str(layer)):
		return prefix + str(layer)
	if layer == 1 and names.has(prefix):  # single-layer URDF
		return prefix
	return ""


func _physics_process(_delta: float) -> void:
	if robot == null or not robot.links.has("pen"):
		return
	if pen_down:
		var p := get_pen_tip()
		p.y = paper_height + 0.0005  # draw just above the paper
		if _trace.is_empty() or _trace[-1].distance_to(p) > 0.0005:
			_trace.append(p)
			if _trace.size() > max_trace_points:
				_trace = _trace.slice(_trace.size() - max_trace_points)
	_trace_mesh.clear_surfaces()
	if _trace.size() >= 2:
		_trace_mesh.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)
		for p in _trace:
			_trace_mesh.surface_add_vertex(p)
		_trace_mesh.surface_end()
