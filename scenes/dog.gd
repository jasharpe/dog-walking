extends CharacterBody3D
class_name Dog

@export var man: Man
@export var leash_length: float
@onready var neck: Node3D = $Model/RootNode/AnimalArmature/Skeleton3D/Neck/NeckNode
@onready var ap: AnimationPlayer = $"Model/AnimationPlayer"
@onready var model: Node3D = $Model

var ANIM_IDLE: String = "AnimalArmature|AnimalArmature|AnimalArmature|Idle"
var ANIM_WALK: String = "AnimalArmature|AnimalArmature|AnimalArmature|Walk"

# Dog movement parameters
var base_movement_speed: float = 3.0
var movement_acceleration: float = 15.0
var movement_friction: float = 15.0

# Leash physics parameters
var leash_force_multiplier: float = 8.0  # How strongly the leash pulls the dog
var dog_resistance: float = 0.3  # How much the dog can resist (0-1, where 1 = equal to man)

# CharacterBody3D physics parameters
var gravity: float = ProjectSettings.get_setting("physics/3d/default_gravity")

# Dog AI behavior (simple wandering when not being pulled)
var wander_target: Vector3
var wander_timer: float = 0.0
var wander_change_interval: float = 2.0

func _ready() -> void:
	# Initialize wander target
	wander_target = global_position + Vector3(randf_range(-2, 2), 0, randf_range(-2, 2))

func _physics_process(delta: float) -> void:
	handle_leash_physics(delta)
	handle_gravity(delta)
	handle_animation()
	move_and_slide()

func handle_leash_physics(delta: float) -> void:
	if man == null:
		return
	
	# Calculate distance and slack
	var man_to_dog = global_position - man.global_position
	var distance = man_to_dog.length()
	var slack = leash_length - distance
	
	var target_velocity = Vector3.ZERO
	
	if slack < 0:  # Leash is taut - dog gets pulled
		# Calculate leash force direction (toward man)
		var leash_direction = -man_to_dog.normalized()
		
		# Leash force increases with tension
		var tension_strength = min(abs(slack) / leash_length, 1.0)
		var leash_force = leash_direction * leash_force_multiplier * tension_strength
		
		# Dog's resistance - tries to maintain some of its desired movement
		var dog_desired_movement = get_dog_desired_movement()
		var resistance_force = dog_desired_movement * dog_resistance
		
		# Combined force (leash dominates, but dog has some influence)
		target_velocity = leash_force + resistance_force
		
		# Ensure we don't exceed reasonable speeds
		if target_velocity.length() > base_movement_speed * 2.0:
			target_velocity = target_velocity.normalized() * base_movement_speed * 2.0
	
	else:  # Leash has slack - dog can move freely
		target_velocity = get_dog_desired_movement() * base_movement_speed
	
	# Apply movement with acceleration
	velocity.x = move_toward(velocity.x, target_velocity.x, movement_acceleration * delta)
	velocity.z = move_toward(velocity.z, target_velocity.z, movement_acceleration * delta)
	
	# Rotate model to face movement direction
	var horizontal_velocity = Vector3(velocity.x, 0, velocity.z)
	if horizontal_velocity.length() > 0.1:
		var target_rotation = atan2(horizontal_velocity.x, horizontal_velocity.z)
		model.rotation.y = lerp_angle(model.rotation.y, target_rotation, 8.0 * delta)

func get_dog_desired_movement() -> Vector3:
	# Simple AI: dog wanders around randomly when not constrained
	wander_timer += get_physics_process_delta_time()
	
	if wander_timer >= wander_change_interval:
		wander_timer = 0.0
		# Pick a new random direction to wander
		var angle = randf() * TAU
		var distance = randf_range(1.0, 3.0)
		wander_target = global_position + Vector3(cos(angle) * distance, 0, sin(angle) * distance)
	
	# Move toward wander target
	var to_target = wander_target - global_position
	to_target.y = 0  # Keep movement horizontal
	
	if to_target.length() > 0.5:
		return to_target.normalized()
	else:
		return Vector3.ZERO

func handle_gravity(delta: float) -> void:
	# Add gravity
	if not is_on_floor():
		velocity.y -= gravity * delta

func handle_animation() -> void:
	var horizontal_velocity = Vector3(velocity.x, 0, velocity.z)
	var anim = ANIM_IDLE
	
	if horizontal_velocity.length() > 0.1:
		anim = ANIM_WALK
	
	if ap.current_animation != anim:
		ap.play(anim, 0.1)
