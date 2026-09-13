extends Node

var jog: JointJogController

func _ready():
	jog = JointJogController.new()

func _process(_delta):
	jog.spin_some()
	jog.set_joint_velocity(["joint_1"], [0.5])

	if jog.has_new_data():
		pass
		#print("Got: ", jog.get_joint_names(), " -> ", jog.get_joint_velocities())
