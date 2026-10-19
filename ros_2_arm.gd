extends Node
class_name RosArmBridge

@export var robot_controller: RobotJointController
@export var robot_name: String = "Robot"

@export var max_joint_velocity: float = 1.0
@export var limit_margin: float = 0.0

@export var joint_sign_overrides: Dictionary[String, float] = {}

enum Mode { SIMULATION, SHADOW }
@export var mode: Mode = Mode.SIMULATION

@export var connection_timeout_sec: float = 1.0
@export var sign_flip := false

var jog: JointJogController
var state_pub: JointStatePublisher
var state_sub: JointStateSubscriber
var _last_velocities: Dictionary[String, float] = {}
var _last_command_time_msec: int = -999999
var _last_shadow_time_msec: int = -999999

func _ready():
	jog = JointJogController.new()
	state_pub = JointStatePublisher.new()
	state_sub = JointStateSubscriber.new()

func toggle_mode() -> void:
	set_mode(Mode.SHADOW if mode == Mode.SIMULATION else Mode.SIMULATION)

func set_mode(new_mode: Mode) -> void:
	mode = new_mode
	# Stop any leftover jog motion when entering shadow
	if mode == Mode.SHADOW:
		for name in robot_controller.get_joint_names():
			robot_controller.set_joint_velocity(name, 0.0)
			_last_velocities[name] = 0.0

func mode_name() -> String:
	return "Simulation" if mode == Mode.SIMULATION else "Shadow"

## True if a /JointJog message has arrived within connection_timeout_sec.
func is_online() -> bool:
	return (Time.get_ticks_msec() - _last_command_time_msec) / 1000.0 < connection_timeout_sec

## True if real-robot joint states have arrived within connection_timeout_sec.
func is_shadow_online() -> bool:
	return (Time.get_ticks_msec() - _last_shadow_time_msec) / 1000.0 < connection_timeout_sec

func _global_sign() -> float:
	return -1.0 if sign_flip else 1.0

func _process(_delta):
	jog.spin_some()
	state_pub.spin_some()
	state_sub.spin_some()

	if mode == Mode.SIMULATION:
		_handle_jog()
		_publish_state()
	else:
		_handle_shadow()

func _handle_jog() -> void:
	if not jog.has_new_data():
		return
	_last_command_time_msec = Time.get_ticks_msec()
	var names = jog.get_joint_names()
	var vels = jog.get_joint_velocities()

	for i in names.size():
		var joint_sign: float = joint_sign_overrides.get(names[i], 1.0)
		var v = clamp(vels[i], -1.0, 1.0) * max_joint_velocity * joint_sign
		var angle = robot_controller.get_joint_angle(names[i]) * joint_sign
		var limits = robot_controller.get_joint_limits(names[i])

		if is_finite(limits.x) and is_finite(limits.y):
			if v > 0.0 and angle >= limits.y - limit_margin:
				v = 0.0
			elif v < 0.0 and angle <= limits.x + limit_margin:
				v = 0.0

		robot_controller.set_joint_velocity(names[i], v)
		_last_velocities[names[i]] = v

func _handle_shadow() -> void:
	if not state_sub.has_new_data():
		return
	_last_shadow_time_msec = Time.get_ticks_msec()
	var names = state_sub.get_joint_names()
	var positions = state_sub.get_joint_positions()
	var s := _global_sign()

	for i in names.size():
		# Undo the same flip used when publishing, so the mapping is symmetric
		robot_controller.set_joint_angle(names[i], positions[i] * s)

func _publish_state() -> void:
	var joint_names := robot_controller.get_joint_names()
	var angles := robot_controller.get_all_angles()
	var positions: Array[float] = []
	var velocities: Array[float] = []
	var s := _global_sign()
	for name in joint_names:
		positions.append(angles[name] * s)
		velocities.append(_last_velocities.get(name, 0.0) * s)
	state_pub.publish_joint_state(joint_names, positions, velocities)
