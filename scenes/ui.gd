extends Control

var objective_scene: Resource = load("res://scenes/objective_label.tscn")

@onready var town: Node3D = $Town
@onready var objective_list: VBoxContainer = $MarginContainer/ObjectiveList

func _ready() -> void:
	for objective in town.objectives:
		var objective_label := objective_scene.instantiate() as ObjectiveLabel
		objective_list.add_child(objective_label)
		var set_objective_label = func() -> void:
			objective_label.text = "☐ " + objective.text + " " + "(" + str(objective.done) + "/" + str(objective.total) + ")"
			if objective.complete:
				objective_label.text = "[color=#00FF7F]" + objective_label.text + "[/color]"
		objective.updated.connect(set_objective_label)
		set_objective_label.call()

func _process(delta: float) -> void:
	pass
	#if Input.is_action_just_pressed("ui_accept"):
		#get_tree().paused = not get_tree().paused
