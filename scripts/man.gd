extends CharacterBody3D
class_name Man

@export var dog: Dog
@export var leash_length: float
@onready var hand: BoneAttachment3D = $Model/RootNode/CharacterArmature/Skeleton3D/Hand
@onready var camera_3d: Camera3D = $Camera3D
@onready var ap: AnimationPlayer = $"Model/AnimationPlayer"
@onready var model: Node3D = $Model
@onready var heel_point: Node3D = $HeelPoint

const ANIM_IDLE: String = "CharacterArmature|CharacterArmature|CharacterArmature|Idle_Hold"
const ANIM_WALK: String = "CharacterArmature|CharacterArmature|CharacterArmature|Walk_Hold"
const ANIM_RUN: String = "CharacterArmature|CharacterArmature|CharacterArmature|Run_Hold"

# Camera orbit parameters
var orbit_distance: float = 5.0
var orbit_yaw: float = PI
var orbit_pitch: float = -0.7  # slight downward angle
var mouse_sensitivity: float = 0.01
var is_rotating_camera: bool = false

# Movement parameters
var base_movement_speed: float = 5.0
var run_speed_multiplier: float = 1.5
var movement_acceleration: float = 20.0
var movement_friction: float = 20.0

# CharacterBody3D physics parameters
var gravity: float = ProjectSettings.get_setting("physics/3d/default_gravity")
var jump_velocity: float = 4.5

func _physics_process(delta: float) -> void:
	handle_movement(delta)
	handle_physics(delta)
	update_camera()

func handle_physics(delta: float) -> void:
	# Add gravity
	if not is_on_floor():
		velocity.y -= gravity * delta
	
	# Handle jump (optional - remove if not needed)
	if Input.is_action_just_pressed("ui_accept") and is_on_floor():
		velocity.y = jump_velocity
	
	# Move the character body
	move_and_slide()

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
	
	# Check if running (holding Shift)
	var is_running = Input.is_key_pressed(KEY_SHIFT)
	var current_speed = base_movement_speed
	if is_running:
		current_speed *= run_speed_multiplier
	
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
	
	# Calculate leash physics
	var effective_dir = movement_dir
	var speed_multiplier = 1.0
	var leash_pull_force = Vector3.ZERO

	if dog != null:
		# Calculate distance between man and dog
		var dog_to_man = global_position - dog.global_position
		var distance = dog_to_man.length()
		
		# Calculate slack (positive = slack, negative = tension)
		var slack = leash_length - distance
		
		if slack < 0:  # Leash is taut
			# Calculate the direction the leash is pulling (from man toward dog)
			var leash_direction = -dog_to_man.normalized()
			
			# Calculate leash pull force based on tension
			var tension_strength = min(abs(slack) / leash_length, 1.0)
			var pull_strength = 2000.0  # Adjust this value to control pull force
			leash_pull_force = leash_direction * tension_strength * pull_strength
			
			if input_dir.length() > 0:
				# Player is moving - apply existing leash constraint logic
				var movement_along_leash = movement_dir.dot(leash_direction)
				
				# Adjust speed based on leash tension and movement direction
				if movement_along_leash > 0:
					# Moving toward dog (with the leash tension) - speed up
					speed_multiplier = lerp(1.0, 1.5, tension_strength)
				else:
					# Moving away from dog (against the leash tension) - slow down
					speed_multiplier = lerp(1.0, 0.5, tension_strength)
				
				# Modify movement direction to account for leash constraint
				var perpendicular_component = movement_dir - (movement_dir.dot(leash_direction) * leash_direction)
				var parallel_component = movement_dir.dot(leash_direction) * leash_direction
				
				# Reduce perpendicular movement when leash is very taut
				var tension_factor = min(abs(slack) / (leash_length * 0.1), 1.0)
				effective_dir = parallel_component + perpendicular_component * (1.0 - tension_factor * 0.5)
				effective_dir = effective_dir.normalized()

	# Apply movement
	if input_dir.length() > 0:
		# Player is actively moving
		var target_velocity = effective_dir * current_speed * speed_multiplier
		velocity.x = move_toward(velocity.x, target_velocity.x, movement_acceleration * delta)
		velocity.z = move_toward(velocity.z, target_velocity.z, movement_acceleration * delta)
		
		# Rotate model to face movement direction
		if effective_dir.length() > 0:
			var target_rotation = atan2(effective_dir.x, effective_dir.z)
			rotation.y = lerp_angle(rotation.y, target_rotation, 10.0 * delta)
	else:
		# Player is not moving - apply friction but also leash pull
		var friction_force = Vector3(velocity.x, 0, velocity.z) * -movement_friction
		var total_force = friction_force + leash_pull_force
		
		# Apply the combined forces
		velocity.x = move_toward(velocity.x, total_force.x * delta, movement_acceleration * delta)
		velocity.z = move_toward(velocity.z, total_force.z * delta, movement_acceleration * delta)
		
		# If being pulled by leash, rotate to face the pull direction
		if leash_pull_force.length() > 0.1:
			var pull_rotation = atan2(leash_pull_force.x, leash_pull_force.z)
			rotation.y = lerp_angle(rotation.y, pull_rotation, 5.0 * delta)
	
	# Handle animation based on horizontal movement and running state
	var horizontal_velocity = Vector3(velocity.x, 0, velocity.z)
	handle_animation(horizontal_velocity, is_running)

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

func handle_animation(current_velocity: Vector3, is_running: bool) -> void:
	var anim = ANIM_IDLE
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
