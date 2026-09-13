extends Node
class_name RosArmBridge

@export var robot_controller: RobotJointController

## The arm's real max joint speed (rad/s) - JointJog sends a normalized
## -1..1 fraction of this. Tune per-robot; Mover6's cpr_ros2 driver used 2.0.
@export var max_joint_velocity: float = 1.0
@export var limit_margin: float = 0.0

## Per-joint sign correction, keyed by joint name, only needed when a joint's
## physical/URDF axis convention is reversed relative to the ROS driver's
## convention. Leave empty for a new arm - fill in only the joints that need
## flipping once you see it moving the wrong way (Mover6 needed all six).
@export var joint_sign_overrides: Dictionary[String, float] = {}

var jog: JointJogController
var state_pub: JointStatePublisher
var _last_velocities: Dictionary[String, float] = {}

func _ready():
	jog = JointJogController.new()
	state_pub = JointStatePublisher.new()

func _process(_delta):
	jog.spin_some()
	state_pub.spin_some()

	if jog.has_new_data():
		var names = jog.get_joint_names()
		var vels = jog.get_joint_velocities()

		for i in names.size():
			var sign_flip = joint_sign_overrides.get(names[i], 1.0)
			var v = clamp(vels[i], -1.0, 1.0) * max_joint_velocity * sign_flip
			var angle = robot_controller.get_joint_angle(names[i]) * sign_flip
			var limits = robot_controller.get_joint_limits(names[i])
			if is_finite(limits.x) and is_finite(limits.y):
				if v > 0.0 and angle >= limits.y - limit_margin:
					v = 0.0
				elif v < 0.0 and angle <= limits.x + limit_margin:
					v = 0.0

			robot_controller.set_joint_velocity(names[i], v)
			_last_velocities[names[i]] = v

	var joint_names := robot_controller.get_joint_names()
	var angles := robot_controller.get_all_angles()
	var positions: Array[float] = []
	var velocities: Array[float] = []
	for name in joint_names:
		#var sign_flip = joint_sign_overrides.get(name, 1.0)
		var sign_flip=1.0
		positions.append(angles[name] * sign_flip)
		velocities.append(_last_velocities.get(name, 0.0) * sign_flip)

	state_pub.publish_joint_state(joint_names, positions, velocities)
