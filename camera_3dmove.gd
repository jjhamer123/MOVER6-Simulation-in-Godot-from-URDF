extends Camera3D
class_name FreeLookCamera

#attach this to a Camera3D node. Click into the viewport to capture the
#mouse look around press Esc to release the mouse again.

@export var move_speed: float = 4.0
@export var sprint_multiplier: float = 3.0
@export var mouse_sensitivity: float = 0.15   # degrees per pixel of mouse motion
@export var vertical_look_limit_deg: float = 89.0

var _yaw: float = 0.0
var _pitch: float = 0.0
var _mouse_captured: bool = false


func _ready() -> void:
	# Initialize yaw/pitch
	var euler := rotation
	_yaw = euler.y
	_pitch = euler.x


func _unhandled_input(event: InputEvent) -> void:
	#click to capture the mouse, Esc to release it.
	if event is InputEventMouseButton and event.pressed and not _mouse_captured:
		_set_mouse_captured(true)
	elif event is InputEventKey and event.pressed and event.keycode == KEY_ESCAPE:
		_set_mouse_captured(false)

	if _mouse_captured and event is InputEventMouseMotion:
		_yaw -= deg_to_rad(event.relative.x * mouse_sensitivity)
		_pitch -= deg_to_rad(event.relative.y * mouse_sensitivity)
		_pitch = clamp(_pitch, deg_to_rad(-vertical_look_limit_deg), deg_to_rad(vertical_look_limit_deg))
		rotation = Vector3(_pitch, _yaw, 0.0)


func _process(delta: float) -> void:
	if not _mouse_captured:
		return
#locally done
	var input_dir := Vector3.ZERO
	if Input.is_key_pressed(KEY_W):
		input_dir -= transform.basis.z
	if Input.is_key_pressed(KEY_S):
		input_dir += transform.basis.z
	if Input.is_key_pressed(KEY_A):
		input_dir -= transform.basis.x
	if Input.is_key_pressed(KEY_D):
		input_dir += transform.basis.x
	if Input.is_key_pressed(KEY_E) or Input.is_key_pressed(KEY_SPACE):
		input_dir += Vector3.UP
	if Input.is_key_pressed(KEY_Q) or Input.is_key_pressed(KEY_CTRL):
		input_dir -= Vector3.UP

#normalise all the same speed
	if input_dir.length_squared() > 0.0:
		input_dir = input_dir.normalized()
		var speed := move_speed
		if Input.is_key_pressed(KEY_SHIFT):
			speed *= sprint_multiplier
		global_translate(input_dir * speed * delta)


func _set_mouse_captured(captured: bool) -> void:
	_mouse_captured = captured
	Input.mouse_mode = Input.MOUSE_MODE_CAPTURED if captured else Input.MOUSE_MODE_VISIBLE
