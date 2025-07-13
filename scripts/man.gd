extends Node3D
class_name Man

@onready var hand: BoneAttachment3D = $Model/RootNode/CharacterArmature/Skeleton3D/Hand

@onready var camera_3d: Camera3D = $Camera3D
@onready var ap: AnimationPlayer = $"Model/AnimationPlayer"
@onready var model: Node3D = $Model

const ANIM_IDLE: String = "CharacterArmature|CharacterArmature|CharacterArmature|Idle_Hold"

# Camera orbit parameters
var orbit_distance: float = 5.0
var orbit_yaw: float = PI
var orbit_pitch: float = -0.7  # slight downward angle
var mouse_sensitivity: float = 0.01
var is_rotating_camera: bool = false

func _process(delta: float) -> void:
	var zoom: float = 2
	var offset = Vector3(
		orbit_distance * cos(orbit_pitch) * sin(orbit_yaw),
		-orbit_distance * sin(orbit_pitch),
		orbit_distance * cos(orbit_pitch) * cos(orbit_yaw)
	)
	camera_3d.global_transform.origin = model.global_transform.origin + zoom * offset
	camera_3d.look_at(model.global_transform.origin + Vector3(0, 1, 0), Vector3.UP)

	if ap.current_animation != ANIM_IDLE:
		ap.play(ANIM_IDLE, 0.1)

func _input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_RIGHT:
			is_rotating_camera = event.pressed
			if is_rotating_camera:
				Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
			else:
				Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
	elif event is InputEventMouseMotion and is_rotating_camera:
		orbit_yaw -= event.relative.x * mouse_sensitivity
		orbit_pitch = clamp(orbit_pitch - event.relative.y * mouse_sensitivity, -1.2, -0.3)
