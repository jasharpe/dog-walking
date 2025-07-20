extends CharacterBody3D
class_name Man

signal bush_checked

@export var dog: Dog
@export var leash: Leash
@export var leash_length: float
@onready var hand: BoneAttachment3D = $Model/RootNode/CharacterArmature/Skeleton3D/Hand
@onready var camera_3d: Camera3D = $Camera3D
@onready var ap: AnimationPlayer = $"Model/AnimationPlayer"
@onready var model: Node3D = $Model
@onready var heel_point: Node3D = $HeelPoint
@onready var interact_radius: Area3D = $InteractRadius

const ANIM_IDLE: String = "CharacterArmature|CharacterArmature|CharacterArmature|Idle_Hold"
const ANIM_WALK: String = "CharacterArmature|CharacterArmature|CharacterArmature|Walk_Hold"
const ANIM_RUN: String = "CharacterArmature|CharacterArmature|CharacterArmature|Run_Hold"
const ANIM_PUNCH: String = "CharacterArmature|CharacterArmature|CharacterArmature|Punch"

# Camera orbit parameters
var orbit_distance: float = 5.0
var orbit_yaw: float = PI
var orbit_pitch: float = -0.7  # slight downward angle
var mouse_sensitivity: float = 0.01
var is_rotating_camera: bool = false

# Movement parameters
var base_movement_force: float = 50.0
var run_force_multiplier: float = 1.5
var movement_acceleration: float = 20.0
var movement_friction: float = 20.0
var man_strength: float = 1.2

# CharacterBody3D physics parameters
var gravity: float = ProjectSettings.get_setting("physics/3d/default_gravity")
var jump_velocity: float = 4.5

var bush: Bush

enum State {
	WALKING,
	CHECKING,
}

var state: State = State.WALKING

func _physics_process(delta: float) -> void:
	if state != State.CHECKING:
		if bush and not bush in interact_radius.get_overlapping_bodies():
			bush.hide_interact()
			bush = null
		for body in interact_radius.get_overlapping_bodies():
			var parent := body.get_parent()
			if parent is Bush and parent.highlighted:
				bush = parent
				break
		if bush:
			bush.show_interact()
	
	if bush and Input.is_action_just_pressed("Check"):
		check()
	
	match state:
		State.WALKING:
			handle_walking(delta)
			handle_physics(delta)
	
	update_camera()

func check() -> void:
	state = State.CHECKING
	ap.play(ANIM_PUNCH, 0.5)
	await ap.animation_finished
	bush.interesting = false
	bush.hide_interact()
	bush = null
	state = State.WALKING
	bush_checked.emit()

func handle_walking(delta: float) -> void:
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
	
	# Check if running (holding Shift)
	var is_running = Input.is_key_pressed(KEY_SHIFT)
	var current_force = base_movement_force * man_strength
	if is_running:
		current_force *= run_force_multiplier
	
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
	
	# Calculate desired movement force from input
	var desired_force = Vector3.ZERO
	if input_dir.length() > 0:
		desired_force = movement_dir * current_force
	
	# Calculate leash force based on connection to first leash segment
	var leash_force = calculate_leash_force()
	
	# Combine forces - man is stronger so his desired movement has priority
	var total_force = desired_force + leash_force
	
	# Apply friction when not inputting movement
	var friction_force = Vector3.ZERO
	if input_dir.length() == 0:
		friction_force = Vector3(velocity.x, 0, velocity.z) * -movement_friction
		total_force += friction_force
	
	# Apply the combined forces to velocity
	var acceleration = total_force / 1.0  # Mass = 1.0 for simplicity
	velocity.x += acceleration.x * delta
	velocity.z += acceleration.z * delta
	
	# Apply movement friction/damping
	velocity.x = lerp(velocity.x, 0.0, movement_friction * 0.1 * delta)
	velocity.z = lerp(velocity.z, 0.0, movement_friction * 0.1 * delta)
	
	# Rotate model to face movement direction
	var horizontal_velocity = Vector3(velocity.x, 0, velocity.z)
	if horizontal_velocity.length() > 0.1:
		var target_rotation = atan2(horizontal_velocity.x, horizontal_velocity.z)
		rotation.y = lerp_angle(rotation.y, target_rotation, 10.0 * delta)
	
	# Handle animation based on horizontal movement and running state
	handle_walking_animation(horizontal_velocity, is_running)

func calculate_leash_force() -> Vector3:
	if not leash or leash.rope_segments.is_empty():
		return Vector3.ZERO
	
	# Get the first segment of the leash (attached to man's hand)
	var first_segment = leash.rope_segments[0]
	var hand_pos = hand.global_position
	var segment_pos = first_segment.global_position
	
	# Calculate the vector from hand to first leash segment
	var to_segment = segment_pos - hand_pos
	var distance = to_segment.length()
	
	# Apply force if there's tension (distance > small threshold)
	var force_threshold = 0.2  # Small threshold to avoid jitter
	if distance > force_threshold:
		var force_direction = to_segment.normalized()
		var force_magnitude = (distance - force_threshold) * 25.0  # Leash stiffness
		return force_direction * force_magnitude
	
	return Vector3.ZERO

func handle_physics(delta: float) -> void:
	# Add gravity
	if not is_on_floor():
		velocity.y -= gravity * delta
	
	# Handle jump (optional - remove if not needed)
	if Input.is_action_just_pressed("ui_accept") and is_on_floor():
		velocity.y = jump_velocity
	
	# Move the character body
	move_and_slide()

func update_camera() -> void:
	var zoom: float = 2
	var offset = Vector3(
		orbit_distance * cos(orbit_pitch) * sin(orbit_yaw),
		-orbit_distance * sin(orbit_pitch),
		orbit_distance * cos(orbit_pitch) * cos(orbit_yaw)
	)
	# Use CharacterBody3D's global_position instead of model's transform
	camera_3d.global_transform.origin = global_position + zoom * offset
	camera_3d.look_at(global_position + Vector3(0, 1, 0), Vector3.UP)

func handle_walking_animation(current_velocity: Vector3, is_running: bool) -> void:
	var anim := ANIM_IDLE
	if current_velocity.length() > 0.05:
		if is_running:
			anim = ANIM_RUN
		else:
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
