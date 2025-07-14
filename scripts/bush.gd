extends Node3D
class_name Bush

@onready var bush: MeshInstance3D = $"Root Scene/RootNode/Bush"
@onready var interact: Label3D = $Interact

var interesting: bool

var UNHIGHLIGHTED_COLOR := Color.from_string("b6e752", Color.GREEN)
var HIGHLIGHTED_COLOR := Color.from_string("e80052", Color.RED)

var highlighted: bool = false
var interactable: bool = false

func _process(delta: float) -> void:
	var camera = get_viewport().get_camera_3d()
	if camera:
		interact.look_at(camera.global_position, Vector3.UP, true)
		interact.rotation.x = 0
		interact.rotation.z = 0

func show_interact() -> void:
	if not interactable:
		interactable = true
		var tween := create_tween()
		tween.tween_property(interact, "modulate:a", 1, 0.2)
		tween.tween_property(interact, "outline_modulate:a", 1, 0.2)

func hide_interact() -> void:
	if interactable:
		interactable = false
		var tween := create_tween()
		tween.tween_property(interact, "modulate:a", 0, 0.2)
		tween.tween_property(interact, "outline_modulate:a", 0, 0.2)

func highlight() -> void:
	# TODO: Tween to change the highlight 
	if not highlighted:
		highlighted = true
		var mat := bush.mesh.surface_get_material(0) as StandardMaterial3D
		var tween := create_tween()
		tween.tween_property(mat, "albedo_color", HIGHLIGHTED_COLOR, 0.5)

func unhighlight() -> void:
	if highlighted:
		highlighted = false
		var mat := bush.mesh.surface_get_material(0) as StandardMaterial3D
		var tween := create_tween()
		tween.tween_property(mat, "albedo_color", UNHIGHLIGHTED_COLOR, 0.5)
