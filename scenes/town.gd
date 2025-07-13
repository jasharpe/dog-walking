extends Node3D

@onready var dog: Dog = $Dog
@onready var man: Man = $Man
@onready var leash: Leash = $Leash

var leash_length: float

func _ready() -> void:
	leash.hand = man.hand
	leash.neck = dog.neck
	leash_length = 1.2 * (leash.hand.global_position - leash.neck.global_position).length()
	leash.leash_length = leash_length
	man.leash_length = leash_length
	dog.leash_length = leash_length
	man.dog = dog
	dog.man = man
