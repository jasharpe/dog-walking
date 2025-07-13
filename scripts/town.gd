extends Node3D

var bush_scene: Resource = load("res://scenes/bush.tscn")

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
	
	# TODO: Randomly spawn 20 instances of bush.tscn around a 50m x 50m square centered at the origin.
	for i in range(20):
		var bush := bush_scene.instantiate() as Bush
		bush.position = Vector3(randf_range(-25, 25), 0, randf_range(-25, 25))
		add_child(bush)
