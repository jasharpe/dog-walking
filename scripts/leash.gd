extends Node3D
class_name Leash

@export var segment_count: int = 10
@export var segment_length: float = 0.5
@export var segment_radius: float = 0.1
@export var rope_mass: float = 1.0
@export var anchor_to_parent: bool = true
@export var rope_stiffness: float = 0.7  # 0-1 range
@export var rope_material: Material
@export var linear_damping: float = 0.5
@export var angular_damping: float = 2.0
@export var stretch_tolerance: float = 0.1  # 10% stretch tolerance
@export var start_node: Node3D = null  # Node to attach start of rope
@export var end_node: Node3D = null    # Node to attach end of rope

@export_group("Visual Settings")
@export var debug_mode: bool = false
@export var leash_color: Color = Color(0.6, 0.4, 0.2)  # Brown color
@export var mesh_resolution: int = 8  # Radial segments for the mesh
@export var curve_smoothness: int = 4  # Points per segment for smooth curve

@export_group("Collision Settings")
@export_flags_3d_physics var rope_collision_layer: int = 1  # What layer the rope is on
@export_flags_3d_physics var rope_collision_mask: int = 1   # What the rope collides with
@export var use_character_exclusion: bool = true  # Auto-exclude character bodies

var rope_segments: Array[RigidBody3D] = []
var joints: Array[PinJoint3D] = []
var attachment_joint: PinJoint3D = null
var start_joint: Node3D = null  # Can be PinJoint3D or Generic6DOFJoint3D
var end_joint: Node3D = null    # Can be PinJoint3D or Generic6DOFJoint3D

# Visual mesh components
var visual_mesh_instance: MeshInstance3D = null
var leash_material: StandardMaterial3D = null

func _ready():
	generate_rope()
	setup_visual_mesh()
	rotation.z = PI / 2
	
	# Connect to nodes if set in inspector
	if start_node:
		attach_start_to_node(start_node)
	if end_node:
		attach_end_to_node(end_node)

func _process(delta: float):
	if rope_segments.size() > 0:
		$Camera3D.position = rope_segments[0].position + Vector3(0, 1, -3)
		$Camera3D.look_at(rope_segments[0].position, Vector3.UP)
	
	# Update visual mesh if not in debug mode
	if not debug_mode:
		update_visual_mesh()

func _physics_process(delta: float):
	# Update rope position if attached to nodes by applying forces
	if start_node and rope_segments.size() > 0:
		var first_segment = rope_segments[0]
		var target_pos = start_node.global_position
		var current_pos = first_segment.global_position
		var distance = current_pos.distance_to(target_pos)
		
		if distance > 0.1:  # Only apply force if significantly far
			var direction = (target_pos - current_pos).normalized()
			# Apply stronger force to pull rope to attachment point
			first_segment.set_linear_velocity(direction * min(distance * 10.0, 20.0))
	
	if end_node and rope_segments.size() > 0:
		var last_segment = rope_segments[-1]
		var target_pos = end_node.global_position
		var current_pos = last_segment.global_position
		var distance = current_pos.distance_to(target_pos)
		
		if distance > 0.1:  # Only apply force if significantly far
			var direction = (target_pos - current_pos).normalized()
			# Apply stronger force to pull rope to attachment point
			last_segment.set_linear_velocity(direction * min(distance * 10.0, 20.0))
	
	# Enforce segment length constraints to prevent excessive stretching
	for i in range(rope_segments.size() - 1):
		var seg_a = rope_segments[i]
		var seg_b = rope_segments[i + 1]
		var distance = seg_a.global_position.distance_to(seg_b.global_position)
		
		if distance > segment_length * (1.0 + stretch_tolerance):
			var direction = (seg_b.global_position - seg_a.global_position).normalized()
			var correction_force = direction * (distance - segment_length) * 10.0
			seg_b.apply_central_impulse(-correction_force * seg_b.mass * delta)
			seg_a.apply_central_impulse(correction_force * seg_a.mass * delta)

func generate_rope():
	clear_existing_rope()
	
	var previous_body: RigidBody3D = null
	
	# Create rope segments
	for i in range(segment_count):
		var segment = create_rope_segment(i)
		rope_segments.append(segment)
		
		# Position segment
		segment.global_position = global_position + Vector3(0, -i * segment_length, 0)
		
		# Create joint to connect segments
		if i > 0:
			var joint = create_joint(previous_body, segment, i)
			joints.append(joint)
		elif anchor_to_parent:
			# Anchor first segment to this node's position
			var anchor_joint = create_anchor_joint(segment)
			joints.append(anchor_joint)
		
		previous_body = segment

func create_rope_segment(index: int) -> RigidBody3D:
	var rigid_body = RigidBody3D.new()
	rigid_body.name = "RopeSegment_" + str(index)
	rigid_body.mass = rope_mass / segment_count  # Distribute mass across segments
	
	# Add physics properties for better rope behavior
	rigid_body.linear_damp = linear_damping
	rigid_body.angular_damp = angular_damping
	rigid_body.continuous_cd = true  # Better collision detection
	rigid_body.can_sleep = false  # Keep rope active
	
	# Set collision layers
	rigid_body.collision_layer = rope_collision_layer
	rigid_body.collision_mask = rope_collision_mask
	
	# Optional: Set to trigger mode for detection without physics push
	# rigid_body.freeze = true  # For Godot 4.x
	# rigid_body.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
	
	# Create collision shape
	var collision_shape = CollisionShape3D.new()
	var capsule_shape = CapsuleShape3D.new()
	capsule_shape.radius = segment_radius
	capsule_shape.height = segment_length
	collision_shape.shape = capsule_shape
	
	# Create visual mesh (only visible in debug mode)
	var mesh_instance = MeshInstance3D.new()
	var capsule_mesh = CapsuleMesh.new()
	capsule_mesh.radius = segment_radius
	# Make segments slightly overlap for visual continuity
	capsule_mesh.height = segment_length * 1.1
	mesh_instance.mesh = capsule_mesh
	mesh_instance.visible = debug_mode
	
	# Apply material if provided
	if rope_material:
		mesh_instance.material_override = rope_material
	
	# Add components to rigid body
	rigid_body.add_child(collision_shape)
	rigid_body.add_child(mesh_instance)
	
	# Add to scene
	add_child(rigid_body)
	
	return rigid_body

func create_joint(body_a: RigidBody3D, body_b: RigidBody3D, index: int) -> PinJoint3D:
	var joint = PinJoint3D.new()
	joint.name = "Joint_" + str(index)
	
	# Add joint to scene first so we can set up paths
	add_child(joint)
	
	# Position joint at the connection point between segments
	# Place it at the bottom of body_a
	joint.global_position = body_a.global_position - Vector3(0, segment_length * 0.5, 0)
	
	# Set up joint connections
	joint.set_node_a(body_a.get_path())
	joint.set_node_b(body_b.get_path())
	
	# Configure joint parameters for better rope behavior
	joint.set_param(PinJoint3D.PARAM_DAMPING, rope_stiffness)
	joint.set_param(PinJoint3D.PARAM_IMPULSE_CLAMP, 0.0)  # No impulse limit
	
	return joint

func create_anchor_joint(body: RigidBody3D) -> PinJoint3D:
	# Create a static body to act as anchor point
	var anchor_body = StaticBody3D.new()
	anchor_body.name = "RopeAnchor"
	get_parent().ready.connect(func() -> void:
		anchor_body.global_position = global_position
	)
	add_child(anchor_body)
	
	# Create joint between anchor and first segment
	var joint = PinJoint3D.new()
	joint.name = "AnchorJoint"
	add_child(joint)
	
	joint.global_position = global_position
	joint.set_node_a(anchor_body.get_path())
	joint.set_node_b(body.get_path())
	
	# Make anchor joint stiffer
	joint.set_param(PinJoint3D.PARAM_DAMPING, rope_stiffness * 2.0)
	joint.set_param(PinJoint3D.PARAM_IMPULSE_CLAMP, 0.0)
	
	return joint

func clear_existing_rope():
	# Remove existing segments
	for segment in rope_segments:
		if is_instance_valid(segment):
			segment.queue_free()
	rope_segments.clear()
	
	# Remove existing joints
	for joint in joints:
		if is_instance_valid(joint):
			joint.queue_free()
	joints.clear()
	
	# Remove anchor if it exists
	var anchor = get_node_or_null("RopeAnchor")
	if anchor:
		anchor.queue_free()
	
	# Clear attachment joints
	if attachment_joint and is_instance_valid(attachment_joint):
		attachment_joint.queue_free()
		attachment_joint = null
	if start_joint and is_instance_valid(start_joint):
		start_joint.queue_free()
		start_joint = null
	if end_joint and is_instance_valid(end_joint):
		end_joint.queue_free()
		end_joint = null

# Utility functions
func get_rope_tip() -> RigidBody3D:
	if rope_segments.size() > 0:
		return rope_segments[-1]
	return null

func apply_force_to_tip(force: Vector3):
	var tip = get_rope_tip()
	if tip:
		tip.apply_central_force(force)

func apply_impulse_to_tip(impulse: Vector3):
	var tip = get_rope_tip()
	if tip:
		tip.apply_central_impulse(impulse)

func set_rope_gravity_scale(scale: float):
	for segment in rope_segments:
		segment.gravity_scale = scale

func update_rope_stiffness():
	for i in range(joints.size()):
		var joint = joints[i]
		if joint is PinJoint3D:
			# Make anchor joint stiffer
			var stiffness = rope_stiffness * 2.0 if i == 0 and anchor_to_parent else rope_stiffness
			joint.set_param(PinJoint3D.PARAM_DAMPING, stiffness)

func attach_to_object(object: Node3D, offset: Vector3 = Vector3.ZERO):
	if rope_segments.is_empty():
		return
	
	# Detach any existing attachment
	detach_from_object()
	
	var tip = get_rope_tip()
	attachment_joint = PinJoint3D.new()
	attachment_joint.name = "AttachmentJoint"
	add_child(attachment_joint)
	
	attachment_joint.global_position = object.global_position + offset
	attachment_joint.set_node_a(tip.get_path())
	attachment_joint.set_node_b(object.get_path())
	
	# Make attachment joint very stiff
	attachment_joint.set_param(PinJoint3D.PARAM_DAMPING, 1.0)
	attachment_joint.set_param(PinJoint3D.PARAM_IMPULSE_CLAMP, 0.0)

func detach_from_object():
	if attachment_joint and is_instance_valid(attachment_joint):
		attachment_joint.queue_free()
		attachment_joint = null

func set_rope_length(new_segment_count: int):
	segment_count = new_segment_count
	regenerate_rope()

# Call this to regenerate the rope with new parameters
func regenerate_rope():
	generate_rope()
	update_rope_stiffness()

# New functions for dual-end attachment
func attach_start_to_node(man: Man, offset: Vector3 = Vector3.ZERO):
	if rope_segments.is_empty():
		return
	
	# Remove existing start attachment
	if start_joint and is_instance_valid(start_joint):
		start_joint.queue_free()
	
	# Remove the anchor joint if it exists
	if anchor_to_parent and joints.size() > 0:
		joints[0].queue_free()
		joints.remove_at(0)
	
	start_node = man.hand
	
	# Move the first segment to the attachment point immediately
	rope_segments[0].global_position = man.hand.global_position + offset
	rope_segments[0].set_linear_velocity(Vector3.ZERO)
	
	# Create a Generic6DOFJoint3D for more control
	var joint = Generic6DOFJoint3D.new()
	joint.name = "StartAttachmentJoint"
	add_child(joint)
	
	joint.global_position = man.hand.global_position + offset
	joint.set_node_a(man.hand.get_path())
	joint.set_node_b(rope_segments[0].get_path())
	
	# Lock all axes for a rigid connection
	for i in range(3):
		joint.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
		joint.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
	
	start_joint = joint
	
	# Exclude collision with character if it's a CharacterBody3D
	if use_character_exclusion and man is CharacterBody3D:
		for segment in rope_segments:
			segment.add_collision_exception_with(man)

func attach_end_to_node(dog: Dog, offset: Vector3 = Vector3.ZERO):
	if rope_segments.is_empty():
		return
	
	# Remove existing end attachment
	if end_joint and is_instance_valid(end_joint):
		end_joint.queue_free()
	
	end_node = dog.neck
	
	# Move the last segment to the attachment point immediately
	var tip = get_rope_tip()
	tip.global_position = dog.neck.global_position + offset
	tip.set_linear_velocity(Vector3.ZERO)
	
	# Create a Generic6DOFJoint3D for more control
	var joint = Generic6DOFJoint3D.new()
	joint.name = "EndAttachmentJoint"
	add_child(joint)
	
	joint.global_position = dog.neck.global_position + offset
	joint.set_node_a(tip.get_path())
	joint.set_node_b(dog.neck.get_path())
	
	# Lock all axes for a rigid connection
	for i in range(3):
		joint.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
		joint.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
		joint.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
		joint.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
	
	end_joint = joint
	
	# Exclude collision with character if it's a CharacterBody3D
	if use_character_exclusion and dog is CharacterBody3D:
		for segment in rope_segments:
			segment.add_collision_exception_with(dog)

func detach_start():
	if start_joint and is_instance_valid(start_joint):
		start_joint.queue_free()
		start_joint = null
		start_node = null
		
		# Restore anchor if needed
		if anchor_to_parent and rope_segments.size() > 0:
			var anchor_joint = create_anchor_joint(rope_segments[0])
			joints.insert(0, anchor_joint)

func detach_end():
	if end_joint and is_instance_valid(end_joint):
		end_joint.queue_free()
		end_joint = null
		end_node = null

func connect_between_nodes(node_a: Node3D, node_b: Node3D):
	"""Convenience function to connect rope between two nodes"""
	attach_start_to_node(node_a)
	attach_end_to_node(node_b)

# Utility function to exclude collision with specific bodies
func add_collision_exception(body: PhysicsBody3D):
	"""Add a body that the rope should not physically interact with"""
	for segment in rope_segments:
		segment.add_collision_exception_with(body)

func remove_collision_exception(body: PhysicsBody3D):
	"""Remove a collision exception"""
	for segment in rope_segments:
		segment.remove_collision_exception_with(body)

func set_rope_as_sensor(is_sensor: bool):
	"""Make rope detect collisions but not apply forces (trigger mode)"""
	for segment in rope_segments:
		if is_sensor:
			# Make segments kinematic to stop them from pushing
			segment.freeze = true
			segment.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
		else:
			segment.freeze = false

func setup_visual_mesh():
	# Create material for the leash
	leash_material = StandardMaterial3D.new()
	leash_material.albedo_color = leash_color
	leash_material.roughness = 0.8
	leash_material.metallic = 0.0
	
	# Create the mesh instance for non-debug mode
	if not debug_mode:
		visual_mesh_instance = MeshInstance3D.new()
		visual_mesh_instance.material_override = leash_material
		# Disable shadow receiving
		#leash_material.flags_do_not_receive_shadows = true
		add_child(visual_mesh_instance)

func update_visual_mesh():
	if not visual_mesh_instance or rope_segments.is_empty():
		return
	
	# Get all segment positions
	var control_points: Array[Vector3] = []
	
	# Add start point (hand position if available)
	if start_node:
		control_points.append(to_local(start_node.global_position))
	
	# Add all segment positions
	for segment in rope_segments:
		control_points.append(to_local(segment.global_position))
	
	# Add end point (neck position if available)
	if end_node:
		control_points.append(to_local(end_node.global_position))
	
	# Generate smooth curve from control points
	if control_points.size() >= 2:
		var smooth_points = generate_smooth_curve(control_points)
		var mesh = generate_rope_mesh(smooth_points)
		visual_mesh_instance.mesh = mesh

func generate_smooth_curve(control_points: Array[Vector3]) -> Array[Vector3]:
	if control_points.size() < 2:
		return control_points
	
	var smooth_points: Array[Vector3] = []
	
	# For each segment between control points, generate smooth interpolated points
	for i in range(control_points.size() - 1):
		var p0 = control_points[max(0, i - 1)]
		var p1 = control_points[i]
		var p2 = control_points[i + 1]
		var p3 = control_points[min(control_points.size() - 1, i + 2)]
		
		# Generate smooth points along this segment using Catmull-Rom spline
		for j in range(curve_smoothness):
			var t = float(j) / float(curve_smoothness)
			var point = catmull_rom_interpolate(p0, p1, p2, p3, t)
			smooth_points.append(point)
	
	# Add the final point
	smooth_points.append(control_points[-1])
	
	return smooth_points

func catmull_rom_interpolate(p0: Vector3, p1: Vector3, p2: Vector3, p3: Vector3, t: float) -> Vector3:
	# Catmull-Rom spline interpolation for smooth curves
	var t2 = t * t
	var t3 = t2 * t
	
	return 0.5 * (
		(2.0 * p1) +
		(-p0 + p2) * t +
		(2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
		(-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
	)

func generate_rope_mesh(points: Array[Vector3]) -> ArrayMesh:
	var array_mesh = ArrayMesh.new()
	var vertices: PackedVector3Array = []
	var normals: PackedVector3Array = []
	var uvs: PackedVector2Array = []
	var indices: PackedInt32Array = []
	
	if points.size() < 2:
		return array_mesh
	
	# Initialize the coordinate frame for the first point
	var forward = (points[1] - points[0]).normalized()
	var up = Vector3.UP
	if abs(forward.dot(up)) > 0.9:
		up = Vector3.RIGHT
	var right = forward.cross(up).normalized()
	up = right.cross(forward).normalized()
	
	# Store the frames for each point to avoid twisting
	var frames: Array = []
	frames.append({"forward": forward, "right": right, "up": up})
	
	# Calculate frames for all subsequent points using parallel transport
	for i in range(1, points.size()):
		var point = points[i]
		var prev_frame = frames[i - 1]
		
		# Calculate new forward direction
		if i == points.size() - 1:
			forward = (points[i] - points[i - 1]).normalized()
		else:
			forward = (points[i + 1] - points[i - 1]).normalized()
		
		# Use parallel transport to minimize twisting
		var prev_forward = prev_frame.forward
		var rotation_axis = prev_forward.cross(forward)
		
		if rotation_axis.length() > 0.001:
			# There's a rotation needed
			rotation_axis = rotation_axis.normalized()
			var rotation_angle = prev_forward.angle_to(forward)
			
			# Rotate the previous right and up vectors
			right = prev_frame.right.rotated(rotation_axis, rotation_angle)
			up = prev_frame.up.rotated(rotation_axis, rotation_angle)
		else:
			# No rotation needed, keep previous orientation
			right = prev_frame.right
			up = prev_frame.up
		
		frames.append({"forward": forward, "right": right, "up": up})
	
	# Generate vertices using the calculated frames
	for i in range(points.size()):
		var point = points[i]
		var frame = frames[i]
		
		# Generate circle of vertices around this point
		var width: float = 0.04
		for j in range(mesh_resolution):
			var angle = j * TAU / mesh_resolution
			var offset = (frame.right * cos(angle) + frame.up * sin(angle)) * width
			var vertex = point + offset
			
			vertices.append(vertex)
			normals.append(offset.normalized())
			uvs.append(Vector2(float(j) / mesh_resolution, float(i) / (points.size() - 1)))
	
	# Generate indices for triangles
	for i in range(points.size() - 1):
		for j in range(mesh_resolution):
			var current_ring = i * mesh_resolution
			var next_ring = (i + 1) * mesh_resolution
			var next_j = (j + 1) % mesh_resolution
			
			# Two triangles per quad (corrected winding order)
			indices.append(current_ring + j)
			indices.append(current_ring + next_j)
			indices.append(next_ring + j)
			
			indices.append(current_ring + next_j)
			indices.append(next_ring + next_j)
			indices.append(next_ring + j)
	
	# Create the mesh
	var arrays = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	arrays[Mesh.ARRAY_INDEX] = indices
	
	array_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return array_mesh

# Debug visualization
func draw_rope_debug():
	# Override this in a tool script to visualize rope in editor
	pass
