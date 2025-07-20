extends Node3D

var bush_scene: Resource = load("res://scenes/bush.tscn")

@onready var dog: Dog = $Dog
@onready var man: Man = $Man
@onready var leash: Leash = $Leash

var leash_length: float

@export var objectives: Array[Objective]

func _ready() -> void:
	#leash.hand = man.hand
	#leash.neck = dog.neck
	#leash_length = 1.2 * (leash.hand.global_position - leash.neck.global_position).length()
	#leash.leash_length = leash_length
	leash.attach_start_to_node(man)
	leash.attach_end_to_node(dog)
	#man.dog = dog
	#dog.man = man
	man.bush_checked.connect(_bush_checked)
	
	# TODO: Randomly spawn 20 instances of bush.tscn around a 50m x 50m square centered at the origin.
	for i in range(20):
		var bush := bush_scene.instantiate() as Bush
		bush.position = Vector3(randf_range(-25, 25), 0, randf_range(-25, 25))
		bush.rotate_y(randf_range(0, TAU))
		if randf_range(0, 1) < 0.5:
			bush.interesting = true
		add_child(bush)

func _bush_checked() -> void:
	var objective = objectives[0]
	if objective.complete:
		return
	objectives[0].done += 1
