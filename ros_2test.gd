extends Node

@export var robot_controller: RobotJointController

var jog: JointJogController
var state_pub: JointStatePublisher
var _last_velocities: Dictionary[String, float] = {}
const MAX_JOINT_VELOCITY := 0.5
const LIMIT_MARGIN := 0.0


func _ready():
	await get_tree().physics_frame
	robot_controller.set_joint_velocity("joint1", 0.2)
	robot_controller.set_joint_velocity("joint2", 0.1)
	robot_controller.set_joint_velocity("joint3", -0.1)
func _process(_delta):
	print("joint1 angle: ", robot_controller.get_joint_angle("joint1"))
