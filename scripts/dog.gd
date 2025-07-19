extends CharacterBody3D
class_name Dog

@export var man: Man
@export var leash_length: float
@onready var neck: Node3D = $Model/RootNode/AnimalArmature/Skeleton3D/Neck/NeckNode
@onready var ap: AnimationPlayer = $"Model/AnimationPlayer"
@onready var model: Node3D = $Model
@onready var alert: Label3D = $Alert
@onready var smell_radius: Area3D = $SmellRadius

var ANIM_IDLE: String = "AnimalArmature|AnimalArmature|AnimalArmature|Idle"
var ANIM_WALK: String = "AnimalArmature|AnimalArmature|AnimalArmature|Walk"
var ANIM_RUN: String = "AnimalArmature|AnimalArmature|AnimalArmature|Run"

# Dog movement parameters
var base_movement_speed: float = 3.0
var movement_acceleration: float = 15.0
var movement_friction: float = 15.0

# Leash physics parameters
var leash_force_multiplier: float = 8.0
var dog_resistance: float = 0.3

# Heel behavior parameters
var heel_distance: float = 1.5
var heel_threshold: float = 0.1
var man_speed_threshold: float = 3
var heel_acceleration: float = 20.0
var heel_max_speed: float = 12.0
var heel_prediction_time: float = 0.2

# CharacterBody3D physics parameters
var gravity: float = ProjectSettings.get_setting("physics/3d/default_gravity")

# Dog AI behavior
var wander_target: Vector3
var wander_timer: float = 0.0
var wander_change_interval: float = 2.0

# Man movement tracking
var man_previous_position: Vector3
var man_velocity: Vector3

# Heel state
var is_heeling: bool = false

func _ready() -> void:
	wander_target = global_position + Vector3(randf_range(-2, 2), 0, randf_range(-2, 2))
	if man:
		man_previous_position = man.global_position

var bush_of_interest: Bush

func _physics_process(delta: float) -> void:
	var bodies := smell_radius.get_overlapping_bodies()
	if bush_of_interest and bush_of_interest.interesting and bush_of_interest in bodies.map(func(body): return body.get_parent()):
		pass
	else:
		if bush_of_interest:
			bush_of_interest.unhighlight()
			bush_of_interest = null
		for body in bodies:
			var parent := body.get_parent()
			if parent is Bush and parent.interesting:
				bush_of_interest = parent
				parent.highlight()
				break
	
	if bush_of_interest:
		alert.modulate.a = 1
		alert.outline_modulate.a = 1
	else:
		alert.modulate.a = 0
		alert.outline_modulate.a = 0
	
	track_man_movement(delta)
	determine_behavior_state(bush_of_interest)
	handle_movement(delta)
	handle_gravity(delta)
	handle_animation()
	move_and_slide()
	
	var camera = get_viewport().get_camera_3d()
	if camera:
		alert.look_at(camera.global_position, Vector3.UP, true)
		alert.rotation.x = 0
		alert.rotation.z = 0

func track_man_movement(delta: float) -> void:
	if man == null:
		return
	
	man_velocity = (man.global_position - man_previous_position) / delta
	man_previous_position = man.global_position

func determine_behavior_state(bush_of_interest: Bush) -> void:
	if man == null:
		is_heeling = false
		return
	
	# Calculate leash tension
	var man_to_dog = global_position - man.global_position
	var distance = man_to_dog.length()
	var slack = leash_length - distance
	var tension_ratio = abs(slack) / leash_length if leash_length > 0 else 0
	
	# Check if conditions are right for heeling
	var man_speed = Vector3(man_velocity.x, 0, man_velocity.z).length()
	var has_tension = slack < 0 and tension_ratio > heel_threshold
	var man_is_moving = man_speed > man_speed_threshold
	
	is_heeling = man_is_moving
	is_heeling = false

func handle_movement(delta: float) -> void:
	var target_velocity = Vector3.ZERO
	var accel = movement_acceleration
	
	if is_heeling:
		target_velocity = get_heel_velocity()
		accel = heel_acceleration
	else:
		target_velocity = get_normal_behavior()
	
	# Apply movement with acceleration
	velocity.x = move_toward(velocity.x, target_velocity.x, accel * delta)
	velocity.z = move_toward(velocity.z, target_velocity.z, accel * delta)
	
	# Rotate model to face movement direction
	var horizontal_velocity = Vector3(velocity.x, 0, velocity.z)
	if horizontal_velocity.length() > 0.1:
		var target_rotation = atan2(horizontal_velocity.x, horizontal_velocity.z)
		rotation.y = lerp_angle(rotation.y, target_rotation, 8.0 * delta)

func get_heel_velocity() -> Vector3:
	# Get target heel position (with prediction)
	var current_heel_pos = man.heel_point.global_position
	var man_horizontal_vel = Vector3(man_velocity.x, 0, man_velocity.z)
	var predicted_heel_pos = current_heel_pos + man_horizontal_vel * heel_prediction_time
	
	# Calculate how far we are from ideal position
	var to_heel = predicted_heel_pos - global_position
	to_heel.y = 0
	var distance_to_heel = to_heel.length()
	
	# If we're very close, just match the man's velocity
	if distance_to_heel < 0.5:
		return man_horizontal_vel
	
	# Otherwise, calculate velocity to reach heel position
	var direction_to_heel = to_heel.normalized()
	
	# Base speed is man's speed, plus extra to catch up
	var base_speed = man_horizontal_vel.length()
	var catch_up_speed = min(distance_to_heel * 3.0, heel_max_speed - base_speed)
	var total_speed = base_speed + catch_up_speed
	
	# Move toward heel position at calculated speed
	var heel_movement = direction_to_heel * total_speed
	
	# Add man's velocity component for smoother following
	var final_velocity = heel_movement * 0.7 + man_horizontal_vel * 0.3
	
	# Clamp to maximum speed
	if final_velocity.length() > heel_max_speed:
		final_velocity = final_velocity.normalized() * heel_max_speed
	
	return final_velocity

func get_normal_behavior() -> Vector3:
	if man == null:
		return get_wander_movement() * base_movement_speed
	
	# Calculate leash physics
	var man_to_dog = global_position - man.global_position
	var distance = man_to_dog.length()
	var slack = leash_length - distance
	
	if slack < 0:  # Leash is taut
		var tension_ratio = abs(slack) / leash_length
		var leash_direction = -man_to_dog.normalized()
		var tension_strength = min(tension_ratio, 1.0)
		var leash_force = leash_direction * leash_force_multiplier * tension_strength
		
		# Add some wandering resistance
		var resistance_force = get_wander_movement() * dog_resistance
		var combined_force = leash_force + resistance_force
		
		# Limit speed
		if combined_force.length() > base_movement_speed * 2.0:
			combined_force = combined_force.normalized() * base_movement_speed * 2.0
		
		return combined_force
	else:
		# Free movement
		return get_wander_movement() * base_movement_speed

func get_wander_movement() -> Vector3:
	wander_timer += get_physics_process_delta_time()
	
	if wander_timer >= wander_change_interval:
		wander_timer = 0.0
		var angle = randf() * TAU
		var distance = randf_range(1.0, 3.0)
		wander_target = global_position + Vector3(cos(angle) * distance, 0, sin(angle) * distance)
	
	var to_target = wander_target - global_position
	to_target.y = 0
	
	if to_target.length() > 0.5:
		return to_target.normalized()
	else:
		return Vector3.ZERO

func handle_gravity(delta: float) -> void:
	if not is_on_floor():
		velocity.y -= gravity * delta

func handle_animation() -> void:
	var horizontal_velocity = Vector3(velocity.x, 0, velocity.z)
	var anim = ANIM_IDLE
	
	if horizontal_velocity.length() > 0.1:
		anim = ANIM_WALK
	if horizontal_velocity.length() > 6:
		anim = ANIM_RUN
	
	if ap.current_animation != anim:
		ap.play(anim, 0.5)
