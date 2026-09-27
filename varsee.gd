extends CanvasLayer
class_name SimOverlay

#have these point at the ROS2 bridge and the arm controller
@export var joint_controller: RobotJointController
@export var ros_bridge: RosArmBridge

#control visability
@export var toggle_key: Key = KEY_TAB
@export var overlay_visibility_key: Key = KEY_H

var _panel: PanelContainer
var _title_label: Label
var _status_label: Label
var _mode_label: Label

var _table_panel: PanelContainer
var _grid: GridContainer
var _joint_names: Array[String] = []
var _angle_labels: Dictionary[String, Label] = {}
var _vel_labels: Dictionary[String, Label] = {}
var _prev_angles: Dictionary[String, float] = {}


func _ready() -> void:
	layer = 10  # draw above the 3D viewport
	_build_status_panel()
	_build_table_skeleton()
	_wait_for_joints_then_populate()

#build panell 
func _build_status_panel() -> void:
	_panel = PanelContainer.new()
	_panel.set_anchors_preset(Control.PRESET_TOP_LEFT)
	_panel.position = Vector2(16, 16)
	_panel.custom_minimum_size = Vector2(260, 0)
	add_child(_panel)

	var margin := MarginContainer.new()
	margin.add_theme_constant_override("margin_left", 12)
	margin.add_theme_constant_override("margin_right", 12)
	margin.add_theme_constant_override("margin_top", 10)
	margin.add_theme_constant_override("margin_bottom", 10)
	_panel.add_child(margin)

	var vbox := VBoxContainer.new()
	margin.add_child(vbox)

	_title_label = Label.new()
	_title_label.add_theme_font_size_override("font_size", 18)
	vbox.add_child(_title_label)

	_status_label = Label.new()
	vbox.add_child(_status_label)

	_mode_label = Label.new()
	vbox.add_child(_mode_label)

	vbox.add_child(HSeparator.new())

	var controls_header := Label.new()
	controls_header.text = "Controls"
	controls_header.add_theme_font_size_override("font_size", 13)
	vbox.add_child(controls_header)

#description needed
	var controls := [
		"[%s]  Toggle Simulation / Shadow" % OS.get_keycode_string(toggle_key),
		"[%s]  Show / hide overlay" % OS.get_keycode_string(overlay_visibility_key),
		"Click viewport  Capture mouse look",
		"[Esc]  Release mouse",
		"[W A S D]  Move camera",
		"[Space / E]  Move up",
		"[Ctrl / Q]  Move down",
		"[Shift]  Sprint",
	]
	for line in controls:
		var hint := Label.new()
		hint.text = line
		hint.add_theme_font_size_override("font_size", 11)
		hint.modulate = Color(1, 1, 1, 0.65)
		vbox.add_child(hint)

#make a table
func _build_table_skeleton() -> void:
	_table_panel = PanelContainer.new()
	_table_panel.set_anchors_preset(Control.PRESET_BOTTOM_LEFT)
	_table_panel.anchor_top = 1.0
	_table_panel.anchor_bottom = 1.0
	_table_panel.grow_vertical = Control.GROW_DIRECTION_BEGIN
	_table_panel.position = Vector2(16, -16)
	add_child(_table_panel)

	#make table contian
	var margin := MarginContainer.new()
	margin.add_theme_constant_override("margin_left", 12)
	margin.add_theme_constant_override("margin_right", 12)
	margin.add_theme_constant_override("margin_top", 10)
	margin.add_theme_constant_override("margin_bottom", 10)
	_table_panel.add_child(margin)

	var vbox := VBoxContainer.new()
	margin.add_child(vbox)

	var title := Label.new()
	title.text = "Joint States"
	title.add_theme_font_size_override("font_size", 14)
	vbox.add_child(title)
	vbox.add_child(HSeparator.new())

	_grid = GridContainer.new()
	_grid.add_theme_constant_override("h_separation", 24)
	_grid.add_theme_constant_override("v_separation", 6)
	vbox.add_child(_grid)


func _wait_for_joints_then_populate() -> void:
	var attempts := 0
	while joint_controller == null or joint_controller.get_joint_names().is_empty():
		#wait until robot loaded
		await get_tree().physics_frame
		attempts += 1
		if attempts > 300:  # ~5s at 60fps, avoid waiting forever if unwired
			push_warning("SimOverlay: no joints found on joint_controller after waiting - is it assigned?")
			return
	_populate_table()


func _populate_table() -> void:
	_joint_names.assign(joint_controller.get_joint_names())
	_grid.columns = _joint_names.size() + 1

	# Row 1: joint names
	_grid.add_child(_row_label(""))
	for joint_name in _joint_names:
		var cell := Label.new()
		cell.text = joint_name
		cell.modulate = Color(1, 1, 1, 0.7)
		_grid.add_child(cell)

	# Row 2: angles
	_grid.add_child(_row_label("Angle (°)"))
	for joint_name in _joint_names:
		var cell := Label.new()
		cell.text = "0.0"
		cell.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
		_grid.add_child(cell)
		_angle_labels[joint_name] = cell
		_prev_angles[joint_name] = joint_controller.get_joint_angle(joint_name)

	# Row 3: velocities
	_grid.add_child(_row_label("Velocity (°/s)"))
	for joint_name in _joint_names:
		var cell := Label.new()
		cell.text = "0.0"
		cell.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
		_grid.add_child(cell)
		_vel_labels[joint_name] = cell

#labels
func _row_label(text: String) -> Label:
	var label := Label.new()
	label.text = text
	label.add_theme_font_size_override("font_size", 12)
	label.modulate = Color(1, 1, 1, 0.7)
	return label


func _unhandled_key_input(event: InputEvent) -> void:
	if not (event is InputEventKey and event.pressed and not event.echo):
		return
	if event.keycode == toggle_key:
		if ros_bridge:
			ros_bridge.toggle_mode()
	elif event.keycode == overlay_visibility_key:
		visible = not visible


func _process(delta: float) -> void:
	if ros_bridge:
		_title_label.text = ros_bridge.robot_name
		_status_label.text = "● Connected" if ros_bridge.is_online() else "○ No signal"
		_status_label.modulate = Color(0.4, 1.0, 0.4) if ros_bridge.is_online() \
			else Color(1.0, 0.4, 0.4)
		_mode_label.text = "Mode: " + ros_bridge.mode_name()
		_mode_label.modulate = Color(0.5, 0.8, 1.0) if ros_bridge.mode == RosArmBridge.Mode.SIMULATION \
			else Color(1.0, 0.8, 0.3)

	if delta <= 0.0:
		return

	for joint_name in _angle_labels.keys():
		var angle := joint_controller.get_joint_angle(joint_name)
		var prev: float = _prev_angles.get(joint_name, angle)
		var speed := (angle - prev) / delta
		_prev_angles[joint_name] = angle

		_angle_labels[joint_name].text = "%.1f" % rad_to_deg(angle)
		_vel_labels[joint_name].text = "%.2f" % rad_to_deg(speed)
