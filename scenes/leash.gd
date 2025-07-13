extends Node3D
class_name Leash

@export var man: Node3D
@export var hand: Node3D
@export var neck: Node3D
@export var leash_length: float = 2.0
@export var rope_radius: float = 0.02
@export var segments: int = 20
@export var man_radius: float = 0.5  # Approximate radius of the man's collision cylinder
@export var wrap_height_offset: float = 0.3  # How much to offset wrap points vertically
@export var wrap_detail_multiplier: float = 2.0  # How much extra detail to give wrap segments

var rope_mesh: MeshInstance3D

func _ready():
	rope_mesh = MeshInstance3D.new()
	add_child(rope_mesh)
	
	# Create a simple material for the rope
	var material = StandardMaterial3D.new()
	material.albedo_color = Color(0.6, 0.4, 0.2)  # Brown leather color
	rope_mesh.material_override = material

func _process(_delta):
	if hand and neck and man:
		update_leash()

func update_leash():
	var start_pos = hand.global_position
	var end_pos = neck.global_position
	
	# Check if the direct line intersects with the man
	var path_points = calculate_leash_path(start_pos, end_pos)
	
	# Debug: Print path points
	if path_points.size() < 2:
		print("Error: Not enough path points: ", path_points.size())
		return
	
	# Generate curve points with sag
	var points = generate_rope_points_with_path(path_points)
	
	# Debug: Check if we have enough points
	if points.size() < 2:
		print("Error: Not enough rope points: ", points.size())
		return
	
	# Create mesh from points
	var mesh = create_rope_mesh(points)
	if mesh == null:
		print("Error: Failed to create mesh")
		return
	
	rope_mesh.mesh = mesh

func calculate_leash_path(start: Vector3, end: Vector3) -> Array[Vector3]:
	var path: Array[Vector3] = []
	path.append(start)
	
	# Check if direct line collides with man
	if does_line_intersect_cylinder(start, end, man.global_position, man_radius):
		# Calculate wrap points around the man
		var wrap_points = calculate_wrap_points(start, end, man.global_position, man_radius)
		path.append_array(wrap_points)
	
	path.append(end)
	return path

func does_line_intersect_cylinder(start: Vector3, end: Vector3, cylinder_center: Vector3, cylinder_radius: float) -> bool:
	# Convert to 2D problem (ignore Y for cylinder collision)
	var start_2d = Vector2(start.x, start.z)
	var end_2d = Vector2(end.x, end.z)
	var center_2d = Vector2(cylinder_center.x, cylinder_center.z)
	
	# Calculate distance from cylinder center to line segment
	var line_vec = end_2d - start_2d
	var line_length = line_vec.length()
	
	if line_length == 0:
		return start_2d.distance_to(center_2d) < cylinder_radius
	
	var line_dir = line_vec / line_length
	var to_center = center_2d - start_2d
	
	# Project center onto line
	var projection = to_center.dot(line_dir)
	projection = clamp(projection, 0, line_length)
	
	var closest_point = start_2d + line_dir * projection
	var distance_to_line = closest_point.distance_to(center_2d)
	
	return distance_to_line < cylinder_radius

func calculate_wrap_points(start: Vector3, end: Vector3, center: Vector3, radius: float) -> Array[Vector3]:
	var wrap_points: Array[Vector3] = []
	
	# Work in 2D for wrapping calculation
	var start_2d = Vector2(start.x, start.z)
	var end_2d = Vector2(end.x, end.z)
	var center_2d = Vector2(center.x, center.z)
	
	# Calculate tangent points from start and end to the circle
	var start_tangents = get_tangent_points(start_2d, center_2d, radius)
	var end_tangents = get_tangent_points(end_2d, center_2d, radius)
	
	# Determine which tangent points to use based on which side of the man they're on
	var start_to_center = center_2d - start_2d
	var end_to_center = center_2d - end_2d
	
	# Use cross product to determine which tangent points create the shorter wrap
	var cross_product = start_to_center.x * end_to_center.y - start_to_center.y * end_to_center.x
	
	var start_tangent: Vector2
	var end_tangent: Vector2
	
	if cross_product > 0:
		# Counter-clockwise wrapping
		start_tangent = start_tangents[0]  # Right tangent from start
		end_tangent = end_tangents[1]     # Left tangent to end
	else:
		# Clockwise wrapping
		start_tangent = start_tangents[1]  # Left tangent from start
		end_tangent = end_tangents[0]     # Right tangent to end
	
	# Calculate smooth height interpolation for wrap points
	# Use 1/3 and 2/3 positions along the straight line for height reference
	var height_at_first_wrap = lerp(start.y, end.y, 0.33)
	var height_at_second_wrap = lerp(start.y, end.y, 0.67)
	
	# Apply a small upward offset for more natural appearance
	height_at_first_wrap += wrap_height_offset
	height_at_second_wrap += wrap_height_offset
	
	# Convert back to 3D and add wrap points
	wrap_points.append(Vector3(start_tangent.x, height_at_first_wrap, start_tangent.y))
	wrap_points.append(Vector3(end_tangent.x, height_at_second_wrap, end_tangent.y))
	
	return wrap_points

func get_tangent_points(point: Vector2, center: Vector2, radius: float) -> Array[Vector2]:
	var tangents: Array[Vector2] = []
	
	var to_point = point - center
	var distance = to_point.length()
	
	if distance <= radius:
		# Point is inside circle, return the point itself
		tangents.append(point)
		tangents.append(point)
		return tangents
	
	# Calculate tangent length
	var tangent_length = sqrt(distance * distance - radius * radius)
	
	# Normalize vector to point
	var to_point_normalized = to_point / distance
	
	# Calculate angle to tangent points
	var angle = acos(radius / distance)
	
	# Calculate the two tangent points
	var angle1 = atan2(to_point_normalized.y, to_point_normalized.x) + angle
	var angle2 = atan2(to_point_normalized.y, to_point_normalized.x) - angle
	
	tangents.append(center + Vector2(cos(angle1), sin(angle1)) * radius)
	tangents.append(center + Vector2(cos(angle2), sin(angle2)) * radius)
	
	return tangents

func generate_rope_points_with_path(path_points: Array[Vector3]) -> Array[Vector3]:
	var all_points: Array[Vector3] = []
	
	# Safety check
	if path_points.size() < 2:
		print("Error: Path points too few: ", path_points.size())
		return all_points
	
	# If only 2 points (no wrapping), use the original simple generation
	if path_points.size() == 2:
		var start = path_points[0]
		var end = path_points[1]
		var sag_amount = calculate_sag_amount(start, end)
		return generate_rope_points(start, end, sag_amount)
	
	# Calculate segment lengths and total path length
	var segment_lengths: Array[float] = []
	var total_length = 0.0
	
	for i in range(path_points.size() - 1):
		var length = path_points[i].distance_to(path_points[i + 1])
		segment_lengths.append(length)
		total_length += length
	
	# Calculate sag amount based on total path length
	var total_sag = calculate_sag_amount_for_path(path_points, total_length)
	
	# Generate points for each segment with smooth height interpolation
	var cumulative_distance = 0.0
	
	for i in range(path_points.size() - 1):
		var start = path_points[i]
		var end = path_points[i + 1]
		var segment_length = segment_lengths[i]
		
		# Determine segment count based on whether this is a wrap segment or not
		var segment_count: int
		if path_points.size() > 2 and i > 0 and i < path_points.size() - 2:
			# This is a wrap segment (middle segments) - give it more detail
			segment_count = max(8, int(segments * 0.4 * wrap_detail_multiplier))
		else:
			# This is a regular segment (first or last) - normal detail
			var length_ratio = segment_length / total_length
			segment_count = max(4, int(segments * length_ratio))
		
		# Generate points for this segment
		for j in range(segment_count + 1):
			if i > 0 and j == 0:
				continue  # Skip first point of non-first segments to avoid duplication
			
			var t = float(j) / float(segment_count)
			var point = start.lerp(end, t)
			
			# Calculate smooth height interpolation across entire path
			var segment_distance = t * segment_length
			var global_t = (cumulative_distance + segment_distance) / total_length
			
			# Create smooth height transition from start to end
			var start_height = path_points[0].y
			var end_height = path_points[path_points.size() - 1].y
			var target_height = lerp(start_height, end_height, global_t)
			
			# Apply sag based on global position along entire path
			var sag_factor = 4.0 * global_t * (1.0 - global_t)  # Parabolic curve
			var final_height = target_height - total_sag * sag_factor
			
			# Set the final height
			point.y = final_height
			
			all_points.append(point)
		
		cumulative_distance += segment_length
	
	return all_points

func calculate_sag_amount_for_path(path_points: Array[Vector3], total_path_length: float) -> float:
	# Calculate sag based on the straight-line distance vs path length
	var straight_distance = path_points[0].distance_to(path_points[path_points.size() - 1])
	var excess_length = total_path_length - straight_distance
	
	# Base sag calculation
	var base_sag = max(0, (leash_length - straight_distance) * 0.3)
	
	# Additional sag for wrapped paths (since they're longer)
	var wrap_sag = excess_length * 0.1
	
	return base_sag + wrap_sag

func generate_rope_points(start: Vector3, end: Vector3, sag: float) -> Array[Vector3]:
	var points: Array[Vector3] = []
	
	for i in range(segments + 1):
		var t = float(i) / float(segments)
		
		# Linear interpolation between start and end
		var point = start.lerp(end, t)
		
		# Add parabolic sag (downward curve)
		var sag_factor = 4.0 * t * (1.0 - t)  # Parabolic curve (peaks at t=0.5)
		point.y -= sag * sag_factor
		
		points.append(point)
	
	return points

func calculate_sag_amount(start: Vector3, end: Vector3) -> float:
	var distance = start.distance_to(end)
	return max(0, (leash_length - distance) * 0.3)

func create_rope_mesh(points: Array[Vector3]) -> ArrayMesh:
	# Safety checks
	if points.size() < 2:
		print("Error: Not enough points for mesh: ", points.size())
		return null
	
	var arrays = []
	arrays.resize(Mesh.ARRAY_MAX)
	
	var vertices: PackedVector3Array = []
	var normals: PackedVector3Array = []
	var uvs: PackedVector2Array = []
	var indices: PackedInt32Array = []
	
	# Create vertices around each point
	var radial_segments = 8
	
	for i in range(points.size()):
		var point = points[i]
		
		# Get direction for this segment
		var direction: Vector3
		if i == 0:
			if points.size() > 1:
				direction = (points[1] - points[0]).normalized()
			else:
				direction = Vector3.FORWARD
		elif i == points.size() - 1:
			direction = (points[i] - points[i-1]).normalized()
		else:
			direction = (points[i+1] - points[i-1]).normalized()
		
		# Ensure direction is not zero
		if direction.length() < 0.001:
			direction = Vector3.FORWARD
		
		# Create perpendicular vectors
		var up = Vector3.UP
		if abs(direction.dot(up)) > 0.9:
			up = Vector3.RIGHT
		
		var right = direction.cross(up).normalized()
		up = right.cross(direction).normalized()
		
		# Create circle of vertices around this point
		for j in range(radial_segments):
			var angle = j * TAU / radial_segments
			var offset = (right * cos(angle) + up * sin(angle)) * rope_radius
			
			vertices.append(point + offset)
			normals.append(offset.normalized())
			uvs.append(Vector2(float(j) / radial_segments, float(i) / points.size()))
	
	# Create indices for triangles
	for i in range(points.size() - 1):
		for j in range(radial_segments):
			var current = i * radial_segments + j
			var next = i * radial_segments + ((j + 1) % radial_segments)
			var current_next_row = (i + 1) * radial_segments + j
			var next_next_row = (i + 1) * radial_segments + ((j + 1) % radial_segments)
			
			# First triangle
			indices.append(current)
			indices.append(next)
			indices.append(current_next_row)
			
			# Second triangle
			indices.append(next)
			indices.append(next_next_row)
			indices.append(current_next_row)
	
	# Safety check for arrays
	if vertices.size() == 0 or indices.size() == 0:
		print("Error: Empty mesh arrays - vertices: ", vertices.size(), " indices: ", indices.size())
		return null
	
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	arrays[Mesh.ARRAY_INDEX] = indices
	
	var mesh = ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh
