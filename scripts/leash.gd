extends Node3D
class_name Leash

@export var hand: Node3D
@export var neck: Node3D
@export var leash_length: float = 2.0
@export var rope_radius: float = 0.02
@export var segments: int = 20

# Physics properties
@export_group("Physics")
@export var segment_mass: float = 0.1
@export var linear_damping: float = 0.8
@export var angular_damping: float = 1.0
@export var joint_stiffness: float = 100.0
@export var joint_damping: float = 10.0

# Visual properties
@export_group("Visual")
@export var rope_material: Material
@export var smoothing_iterations: int = 3

# Internal references
var segment_bodies: Array[RigidBody3D] = []
var joints: Array[Generic6DOFJoint3D] = []
var visual_path: Path3D
var path_follow: PathFollow3D
var mesh_instance: MeshInstance3D

func _ready():
	# Wait for parent to be ready (so hand/neck references are set)
	await get_parent().ready
	
	if not hand or not neck:
		push_error("Leash requires both hand and neck nodes to be set")
		return
	
	# Ensure we're in the tree before accessing global positions
	if not is_inside_tree():
		await tree_entered
	
	setup_rope_segments()
	setup_visual_representation()

func setup_rope_segments():
	var segment_length = leash_length / float(segments)
	var start_pos = hand.global_position
	var end_pos = neck.global_position
	var direction = (end_pos - start_pos).normalized()
	
	# Create segments
	for i in segments:
		var segment = create_segment(i, segment_length)
		
		# Add to scene tree first
		add_child(segment)
		segment_bodies.append(segment)
		
		# Now we can safely set global position
		var t = float(i) / float(segments - 1)
		segment.global_position = start_pos.lerp(end_pos, t)
	
	# Create joints between segments
	for i in segments - 1:
		var joint = create_joint(segment_bodies[i], segment_bodies[i + 1])
		add_child(joint)
		joints.append(joint)
	
	# Attach ends to hand and neck
	attach_to_bone(segment_bodies[0], hand, true)
	attach_to_bone(segment_bodies[-1], neck, false)

func create_segment(index: int, length: float) -> RigidBody3D:
	var segment = RigidBody3D.new()
	segment.name = "LeashSegment" + str(index)
	segment.mass = segment_mass
	segment.linear_damp = linear_damping
	segment.angular_damp = angular_damping
	segment.continuous_cd = true  # Prevent tunneling
	
	# Add collision shape
	var collision = CollisionShape3D.new()
	var capsule = CapsuleShape3D.new()
	capsule.radius = rope_radius
	capsule.height = length
	collision.shape = capsule
	segment.add_child(collision)
	
	# Set collision layers (adjust as needed)
	segment.collision_layer = 4  # Leash layer
	segment.collision_mask = 1 | 2  # Collide with environment and characters
	
	return segment

func create_joint(body_a: RigidBody3D, body_b: RigidBody3D) -> Generic6DOFJoint3D:
	var joint = Generic6DOFJoint3D.new()
	joint.node_a = body_a.get_path()
	joint.node_b = body_b.get_path()
	
	# Configure joint limits and spring properties
	# Allow some movement but keep segments connected
	for axis in ["x", "y", "z"]:
		# Linear limits - small amount of stretch
		joint.set("linear_limit_" + axis + "/enabled", true)
		joint.set("linear_limit_" + axis + "/upper_distance", 0.05)
		joint.set("linear_limit_" + axis + "/lower_distance", -0.05)
		joint.set("linear_limit_" + axis + "/softness", 0.5)
		
		# Angular limits - prevent excessive rotation
		joint.set("angular_limit_" + axis + "/enabled", true)
		joint.set("angular_limit_" + axis + "/upper_angle", deg_to_rad(45))
		joint.set("angular_limit_" + axis + "/lower_angle", deg_to_rad(-45))
		
		# Spring properties for natural movement
		joint.set("linear_spring_" + axis + "/enabled", true)
		joint.set("linear_spring_" + axis + "/stiffness", joint_stiffness)
		joint.set("linear_spring_" + axis + "/damping", joint_damping)
	
	return joint

func attach_to_bone(segment: RigidBody3D, bone: Node3D, is_hand: bool):
	# Store reference for manual position updates
	if is_hand:
		segment.set_meta("attached_to_hand", true)
	else:
		segment.set_meta("attached_to_neck", true)
	
	# Make the end segments kinematic so we can control them directly
	segment.freeze = true
	segment.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC

func setup_visual_representation():
	# Create path for smooth visuals
	visual_path = Path3D.new()
	visual_path.curve = Curve3D.new()
	add_child(visual_path)
	
	# Create mesh instance for the rope
	mesh_instance = MeshInstance3D.new()
	if rope_material:
		mesh_instance.material_override = rope_material
	add_child(mesh_instance)

func _physics_process(_delta):
	# Update end segment positions to follow hand and neck
	if segment_bodies.size() > 0:
		if hand and segment_bodies[0].has_meta("attached_to_hand"):
			segment_bodies[0].global_position = hand.global_position
		if neck and segment_bodies[-1].has_meta("attached_to_neck"):
			segment_bodies[-1].global_position = neck.global_position
	
	update_visual_representation()
	
	# Optional: Apply additional forces for better behavior
	apply_tension_forces()

func update_visual_representation():
	if not visual_path or not visual_path.curve:
		return
	
	var curve = visual_path.curve
	curve.clear_points()
	
	# Get segment positions
	var points: Array[Vector3] = []
	for segment in segment_bodies:
		points.append(segment.global_position)
	
	# Smooth the points
	var smoothed_points = smooth_points(points, smoothing_iterations)
	
	# Add points to curve
	for point in smoothed_points:
		curve.add_point(visual_path.to_local(point))
	
	# Generate rope mesh
	generate_rope_mesh(curve)

func smooth_points(points: Array[Vector3], iterations: int) -> Array[Vector3]:
	var result = points.duplicate()
	
	for _i in iterations:
		var new_points: Array[Vector3] = []
		new_points.append(result[0])  # Keep first point fixed
		
		for j in range(1, result.size() - 1):
			# Average with neighbors
			var smoothed = (result[j-1] + result[j] * 2.0 + result[j+1]) / 4.0
			new_points.append(smoothed)
		
		new_points.append(result[-1])  # Keep last point fixed
		result = new_points
	
	return result

func generate_rope_mesh(curve: Curve3D):
	if curve.point_count < 2:
		return
	
	var arrays = []
	arrays.resize(Mesh.ARRAY_MAX)
	
	var vertices = PackedVector3Array()
	var normals = PackedVector3Array()
	var uvs = PackedVector2Array()
	var indices = PackedInt32Array()
	
	var radial_segments = 8
	var length_accumulated = 0.0
	
	# Generate vertices along the curve
	for i in curve.point_count:
		var point = curve.get_point_position(i)
		var tangent: Vector3
		
		if i == 0:
			tangent = (curve.get_point_position(1) - point).normalized()
		elif i == curve.point_count - 1:
			tangent = (point - curve.get_point_position(i-1)).normalized()
		else:
			tangent = (curve.get_point_position(i+1) - curve.get_point_position(i-1)).normalized()
		
		# Create perpendicular vectors
		var right = tangent.cross(Vector3.UP).normalized()
		if right.length() < 0.01:
			right = tangent.cross(Vector3.RIGHT).normalized()
		var up = right.cross(tangent).normalized()
		
		# Add vertices in a circle
		for j in radial_segments:
			var angle = (j / float(radial_segments)) * TAU
			var offset = (right * cos(angle) + up * sin(angle)) * rope_radius
			vertices.append(point + offset)
			normals.append(offset.normalized())
			uvs.append(Vector2(j / float(radial_segments), length_accumulated))
		
		if i > 0:
			length_accumulated += (point - curve.get_point_position(i-1)).length()
	
	# Generate indices
	for i in curve.point_count - 1:
		for j in radial_segments:
			var current = i * radial_segments + j
			var next = i * radial_segments + (j + 1) % radial_segments
			var current_next = (i + 1) * radial_segments + j
			var next_next = (i + 1) * radial_segments + (j + 1) % radial_segments
			
			# First triangle
			indices.append(current)
			indices.append(next)
			indices.append(next_next)
			
			# Second triangle
			indices.append(current)
			indices.append(next_next)
			indices.append(current_next)
	
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	arrays[Mesh.ARRAY_INDEX] = indices
	
	var array_mesh = ArrayMesh.new()
	array_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	mesh_instance.mesh = array_mesh

func apply_tension_forces():
	# Apply gentle forces to keep rope taut when needed
	var total_length = 0.0
	for i in segment_bodies.size() - 1:
		total_length += segment_bodies[i].global_position.distance_to(segment_bodies[i+1].global_position)
	
	# If rope is significantly stretched, apply corrective forces
	if total_length > leash_length * 1.2:
		for i in range(1, segment_bodies.size() - 1):
			var to_prev = (segment_bodies[i-1].global_position - segment_bodies[i].global_position).normalized()
			var to_next = (segment_bodies[i+1].global_position - segment_bodies[i].global_position).normalized()
			var correction_force = (to_prev + to_next) * 0.5 * segment_mass * 10.0
			segment_bodies[i].apply_central_force(correction_force)

func get_tension() -> float:
	# Calculate how taut the leash is (0.0 = slack, 1.0 = fully extended)
	var total_length = 0.0
	for i in segment_bodies.size() - 1:
		total_length += segment_bodies[i].global_position.distance_to(segment_bodies[i+1].global_position)
	
	return clamp((total_length - leash_length * 0.9) / (leash_length * 0.3), 0.0, 1.0)

func apply_pull_force(from_hand: bool, force: Vector3):
	# Apply force to the appropriate end of the leash
	# Since end segments are kinematic, apply to the next segment inward
	var target_segment_index = 1 if from_hand else segment_bodies.size() - 2
	if target_segment_index >= 0 and target_segment_index < segment_bodies.size():
		segment_bodies[target_segment_index].apply_central_impulse(force)
