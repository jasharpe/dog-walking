extends Node3D
class_name Man

@onready var hand: BoneAttachment3D = $Model/RootNode/CharacterArmature/Skeleton3D/Hand
@onready var camera_3d: Camera3D = $Camera3D
@onready var ap: AnimationPlayer = $"Model/AnimationPlayer"
@onready var model: Node3D = $Model

const ANIM_IDLE: String = "CharacterArmature|CharacterArmature|CharacterArmature|Idle_Hold"
const ANIM_WALK: String = "CharacterArmature|CharacterArmature|CharacterArmature|Walk_Hold"

# Camera orbit parameters
var orbit_distance: float = 5.0
var orbit_yaw: float = PI
var orbit_pitch: float = -0.7  # slight downward angle
var mouse_sensitivity: float = 0.01
var is_rotating_camera: bool = false

# Movement parameters
var movement_speed: float = 5.0
var movement_acceleration: float = 10.0
var movement_friction: float = 8.0
var current_velocity: Vector3 = Vector3.ZERO

func _process(delta: float) -> void:
	handle_movement(delta)
	update_camera()

func handle_movement(delta: float) -> void:
	# Get input
	var input_dir = Vector3.ZERO
	
	if Input.is_action_pressed("ui_right") or Input.is_key_pressed(KEY_D):
		input_dir.x += 1
	if Input.is_action_pressed("ui_left") or Input.is_key_pressed(KEY_A):
		input_dir.x -= 1
	if Input.is_action_pressed("ui_down") or Input.is_key_pressed(KEY_S):
		input_dir.z += 1
	if Input.is_action_pressed("ui_up") or Input.is_key_pressed(KEY_W):
		input_dir.z -= 1
	
	# Convert input to camera-relative movement
	var camera_forward = -camera_3d.global_transform.basis.z
	var camera_right = camera_3d.global_transform.basis.x
	
	# Project vectors onto the horizontal plane (remove Y component)
	camera_forward.y = 0
	camera_right.y = 0
	camera_forward = camera_forward.normalized()
	camera_right = camera_right.normalized()
	
	# Calculate movement direction relative to camera
	var movement_dir = (camera_right * input_dir.x + camera_forward * -input_dir.z).normalized()
	
	# Apply movement with acceleration/friction
	if input_dir.length() > 0:
		current_velocity = current_velocity.move_toward(movement_dir * movement_speed, movement_acceleration * delta)
		
		# Rotate model to face movement direction
		if movement_dir.length() > 0:
			var target_rotation = atan2(movement_dir.x, movement_dir.z)
			model.rotation.y = lerp_angle(model.rotation.y, target_rotation, 10.0 * delta)
	else:
		current_velocity = current_velocity.move_toward(Vector3.ZERO, 3 * movement_friction * delta)
	
	# Apply movement
	model.global_transform.origin += current_velocity * delta
	
	handle_animation(current_velocity)

func update_camera() -> void:
	var zoom: float = 2
	var offset = Vector3(
		orbit_distance * cos(orbit_pitch) * sin(orbit_yaw),
		-orbit_distance * sin(orbit_pitch),
		orbit_distance * cos(orbit_pitch) * cos(orbit_yaw)
	)
	camera_3d.global_transform.origin = model.global_transform.origin + zoom * offset
	camera_3d.look_at(model.global_transform.origin + Vector3(0, 1, 0), Vector3.UP)

func handle_animation(current_velocity: Vector3) -> void:
	var anim = ANIM_IDLE
	if current_velocity.length() > 0.05:
		anim = ANIM_WALK
	if ap.current_animation != anim:
		ap.play(anim, 0.1)

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
