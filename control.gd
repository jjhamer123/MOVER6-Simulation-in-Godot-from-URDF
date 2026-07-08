extends Node

@onready var controller := $"../ArmController" as RobotJointController

func _ready() -> void:
	await get_tree().create_timer(2).timeout

func _process(_delta: float) -> void:
	controller.set_joint_velocity("joint1", 0.3)
	
