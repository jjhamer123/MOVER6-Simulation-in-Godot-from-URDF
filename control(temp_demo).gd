extends Node
class_name JointStateSimulator

@export var simulate: bool = true
@export var joint_names: Array[String] = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
@export var sim_speed: float = 1.0       # oscillation speed
@export var sim_amplitude: float = 0.5   # radians

var publisher: JointStatePublisher
var _t := 0.0

func _ready():
	publisher = JointStatePublisher.new()

func _process(delta):
	publisher.spin_some()

	if simulate:
		_t += delta * sim_speed
		var positions: Array[float] = []
		var velocities: Array[float] = []
		for i in joint_names.size():
			# offset each joint's phase so they don't all move identically
			var phase = i * 0.5
			positions.append(sin(_t + phase) * sim_amplitude)
			velocities.append(cos(_t + phase) * sim_amplitude * sim_speed)
		publisher.publish_joint_state(joint_names, positions, velocities)
