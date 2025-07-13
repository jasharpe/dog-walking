extends Node3D

@onready var dog: Dog = $Dog
@onready var man: Man = $Man
@onready var leash: Leash = $Leash

func _ready() -> void:
	leash.hand = man.hand
	leash.neck = dog.neck
	leash.leash_length = 1.2 * (leash.hand.global_position - leash.neck.global_position).length()
