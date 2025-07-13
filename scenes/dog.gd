extends Node3D
class_name Dog

@export var man: Man
@export var leash_length: float

@onready var neck: BoneAttachment3D = $Model/RootNode/AnimalArmature/Skeleton3D/Neck

@onready var ap: AnimationPlayer = $"Model/AnimationPlayer"
@onready var model: Node3D = $Model

var ANIM_IDLE: String = "AnimalArmature|AnimalArmature|AnimalArmature|Idle"

func _process(delta: float) -> void:
	if ap.current_animation != ANIM_IDLE:
		ap.play(ANIM_IDLE, 0.1)
