extends Node3D
class_name Leash

@export var hand: Node3D
@export var neck: Node3D
@export var leash_length: float = 2.0
@export var rope_radius: float = 0.02
@export var segments: int = 20

var rope_mesh: MeshInstance3D

func _ready():
	rope_mesh = MeshInstance3D.new()
	add_child(rope_mesh)
	
	# Create a simple material for the rope
	var material = StandardMaterial3D.new()
	material.albedo_color = Color(0.6, 0.4, 0.2)  # Brown leather color
	rope_mesh.material_override = material

func _process(_delta):
	if hand and neck:
		update_leash()

func update_leash():
	var start_pos = hand.global_position
	var end_pos = neck.global_position
	var distance = start_pos.distance_to(end_pos)
	
	# Calculate sag amount
	var sag_amount = max(0, (leash_length - distance) * 0.3)
	
	# Generate curve points
	var points = generate_rope_points(start_pos, end_pos, sag_amount)
	
	# Create mesh from points
	var mesh = create_rope_mesh(points)
	rope_mesh.mesh = mesh

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

func create_rope_mesh(points: Array[Vector3]) -> ArrayMesh:
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
			direction = (points[1] - points[0]).normalized()
		elif i == points.size() - 1:
			direction = (points[i] - points[i-1]).normalized()
		else:
			direction = (points[i+1] - points[i-1]).normalized()
		
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
	
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	arrays[Mesh.ARRAY_INDEX] = indices
	
	var mesh = ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh
