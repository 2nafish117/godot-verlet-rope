tool
extends ImmediateGeometry

# references: 
# https://docs.unrealengine.com/4.26/en-US/Basics/Components/Rendering/CableComponent/
# https://owlree.blog/posts/simulating-a-rope.html
# https://qroph.github.io/2018/07/30/smooth-paths-using-catmull-rom-splines.html
# https://toqoz.fyi/game-rope.html

const INV_SQRT_2: float = 1.0 / sqrt(2.0)
const COS_5_DEG: float = cos(deg2rad(5))
const COS_10_DEG: float = cos(deg2rad(10))
const COS_15_DEG: float = cos(deg2rad(15))
const COS_20_DEG: float = cos(deg2rad(20))
const COS_25_DEG: float = cos(deg2rad(25))
const COS_30_DEG: float = cos(deg2rad(30))

export(bool) var attach_start: bool = true setget set_attach_start
func set_attach_start(value: bool) -> void:
	attach_start = value
	if particle_data:
		particle_data.is_attached[0] = value

export(NodePath) var attach_end_to: NodePath setget set_attach_end_to
func set_attach_end_to(val: NodePath) -> void:
	attach_end_to = val 
	if particle_data != null:
		particle_data.is_attached[-1] = is_attached_end()

export(float) var rope_length: float = 5.0
export(float) var rope_width := 0.07
 
export(int, 3, 200) var simulation_particles: int = 9 setget set_simulation_particles
func set_simulation_particles(val: int) -> void:
	simulation_particles = val
	if particle_data:
		particle_data.resize(simulation_particles)
		_create_rope()

export(int) var iterations: int = 2 # low value = more sag, high value = less sag
export(int, 0, 120) var preprocess_iterations: int = 0
export(int, 10, 60) var simulation_rate: int = 60
export(float, 0.01, 1.5) var stiffness = 0.9 # low value = elastic, high value = taut rope

export(bool) var simulate: bool = true
export(bool) var draw: bool = true
export(float) var subdiv_lod_distance = 15.0 # switches to only drawing quads between particles at this point

export(bool) var apply_gravity: bool = true
export(Vector3) var gravity: Vector3 = Vector3.DOWN * 9.8
export(float) var gravity_scale: float = 1.0

export(bool) var apply_wind: bool = false
export(OpenSimplexNoise) var wind_noise: OpenSimplexNoise
export(Vector3) var wind: Vector3 = Vector3(1.0, 0.0, 0.0)
export(float) var wind_scale: float = 10.0

export(bool) var apply_damping: bool = true
export(float) var damping_factor: float = 100.0

export(bool) var apply_collision: bool = false
export(int, LAYERS_3D_PHYSICS) var collision_mask: int = 1
 
func get_end_location() -> Vector3:
	return particle_data.pos_curr[-1]

func is_attached_end() -> bool:
	return not attach_end_to.is_empty()

func is_attached_start() -> bool:
	return attach_start

func get_segment_length() -> float:
	return rope_length / (simulation_particles - 1)

# unused func, maybe useful in code?
func add_particle_at_end(adjust_length: bool) -> void:
	var _pos_prev: Vector3 = particle_data.pos_prev[-1] + Vector3.BACK * 0.01;
	var _pos_curr: Vector3 = particle_data.pos_curr[-1] + Vector3.BACK * 0.01;
	var _is_attached: bool = particle_data.is_attached[-1]
	var _accel: Vector3 = particle_data.accel[-1]
	
	var _tangent: Vector3 = particle_data.tangents[-1]
	var _normal: Vector3 = particle_data.normals[-1]
	var _binormal: Vector3 = particle_data.binormals[-1]
	
	particle_data.is_attached[-1] = false
	
	particle_data.pos_curr.append(_pos_curr)
	particle_data.pos_prev.append(_pos_prev)
	particle_data.accel.append(_accel)
	particle_data.is_attached.append(_is_attached)
	particle_data.tangents.append(_tangent)
	particle_data.normals.append(_normal)
	particle_data.binormals.append(_binormal)
	
	simulation_particles += 1
	if adjust_length:
		rope_length += get_segment_length()

# private
var time: float = 0.0
var particle_data: RopeParticleData
var visibility_notifier: VisibilityNotifier
var collision_check_param: PhysicsShapeQueryParameters
var collision_check_box: BoxShape

onready var space_state = get_world().direct_space_state

class RopeParticleData:
	var pos_curr: PoolVector3Array
	var pos_prev: PoolVector3Array
	var accel: PoolVector3Array
	var is_attached: Array
	
	var tangents: PoolVector3Array
	var normals: PoolVector3Array
	var binormals: PoolVector3Array

	func is_empty() -> bool:
		return len(pos_curr) == 0

	func resize(idx: int) -> void:
		pos_curr.resize(idx)
		pos_prev.resize(idx)
		accel.resize(idx)
		is_attached.resize(idx)
		
		tangents.resize(idx)
		normals.resize(idx)
		binormals.resize(idx)

	func _init() -> void:
		resize(3)

# functions that hardcode some of the catmull function values, for performance
# t = 0.0, 1.0
static func catmull_interpolate_in_step_one(p0: Vector3, p1: Vector3, p2: Vector3, p3: Vector3) -> PoolVector3Array:
	var m1: Vector3 = 0.5 * (p2 - p0)
	var m2: Vector3 = 0.5 * (p3 - p1)
	# order point, tangent ...
	return PoolVector3Array([
		p1, m1.normalized(), # t = 0.0
		p2, m2.normalized() # t = 1.0
	])

# t = 0.0, 0.25, 0.5, 0.75, 1.0
static func catmull_interpolate_in_step_fourths(p0: Vector3, p1: Vector3, p2: Vector3, p3: Vector3) -> PoolVector3Array:
	var m1: Vector3 = 0.5 * (p2 - p0)
	var m2: Vector3 = 0.5 * (p3 - p1)

	# order point, tangent ... 
	return PoolVector3Array([
		p1, m1.normalized(), # t = 0.0
		0.84375 * p1 + 0.15625 * p2 + 0.140625 * m1 - 0.046875 * m2, (-1.125 * p1 + 1.125 * p2 + 0.1875 * m1 - 0.3125 * m2).normalized(), # t = 0.25
		0.5 * p1 + 0.5 * p2 + 0.125 * m1 - 0.125 * m2, (-1.5 * p1 + 1.5 * p2 - 0.25 * m1 - 0.25 * m2).normalized(), # t = 0.5
		0.15625 * p1 + 0.84375 * p2 + 0.046875 * m1 - 0.140625 * m2, (-1.125 * p1 + 1.125 * p2 - 0.3125 * m1 + 0.1875 * m2).normalized(), # t = 0.75
		p2, m2.normalized() # t = 1.0
	])

# t = 0.0, 0.333, 0.6666, 1.0
static func catmull_interpolate_in_step_thirds(p0: Vector3, p1: Vector3, p2: Vector3, p3: Vector3) -> PoolVector3Array:
	var m1: Vector3 = 0.5 * (p2 - p0)
	var m2: Vector3 = 0.5 * (p3 - p1)

	# order point, tangent ...
	return PoolVector3Array([
		p1, m1.normalized(), # t = 0.0
		0.7407407407407407 * p1 + 0.25925925925925924 * p2 + 0.14814814814814814 * m1 - 0.07407407407407407 * m2, (-1.3333333333333335 * p1 + 1.3333333333333335 * p2 - 0.3333333333333333 * m2).normalized(), # t = 0.33 
		0.2592592592592593 * p1 + 0.7407407407407407 * p2 + 0.07407407407407407 * m1 - 0.14814814814814814 * m2, (-1.3333333333333335 * p1 + 1.3333333333333335 * p2 - 0.33333333333333326 * m1).normalized(), # t = 0.66
		p2, m2.normalized() # t = 1.0
	])

# t = 0.0, 0.5, 1.0
static func catmull_interpolate_in_step_halfs(p0: Vector3, p1: Vector3, p2: Vector3, p3: Vector3) -> PoolVector3Array:
	var m1: Vector3 = 0.5 * (p2 - p0)
	var m2: Vector3 = 0.5 * (p3 - p1)

	# order point, tangent
	return PoolVector3Array([
		p1, m1.normalized(), # t = 0.0
		0.5 * p1 + 0.5 * p2 + 0.125 * m1 - 0.125 * m2, (-1.5 * p1 + 1.5 * p2 - 0.25 * m1 - 0.25 * m2).normalized(), # t = 0.5
		p2, m2.normalized() # t = 1.0
	])

# fast? catmull spline
static func catmull_interpolate(p0: Vector3, p1: Vector3, p2: Vector3, p3: Vector3, t: float) -> PoolVector3Array:
	var t_sqr: float = t * t
	var t_cube: float = t_sqr * t

	var m1: Vector3 = 0.5 * (p2 - p0)
	var m2: Vector3 = 0.5 * (p3 - p1)

	var a: Vector3 = 2.0 * (p1 - p2) + m1 + m2
	var b: Vector3 = -3.0 * (p1 - p2) - 2.0 * m1 - m2;
	var c: Vector3 = m1;
	var d: Vector3 = p1;
	# order point, tangent
	return PoolVector3Array([a * t_cube + b * t_sqr + c * t + d, (3.0 * a * t_sqr + 2.0 * b * t + c).normalized()])

# unused func draws simple lines between particles
func _draw_linear_curve():
	begin(Mesh.PRIMITIVE_TRIANGLES)
	for i in range(simulation_particles - 1):
		var curr_pos: Vector3 = particle_data.pos_curr[i] - global_transform.origin
		var curr_binorm: Vector3 = particle_data.binormals[i]
		
		var next_pos: Vector3 = particle_data.pos_curr[i + 1] - global_transform.origin
		var next_binorm: Vector3 = particle_data.binormals[i + 1]
		
		_draw_quad([
			curr_pos - curr_binorm * rope_width, 
			next_pos - next_binorm * rope_width, 
			next_pos + next_binorm * rope_width, 
			curr_pos + curr_binorm * rope_width], 
			-curr_binorm, particle_data.tangents[i], 
			0.0, 1.0,
			Color.black)
	end()

func _draw_interval(data: PoolVector3Array, camera_position: Vector3, step: float, tangent: Vector3) -> void:
	var t: float = 0.0
	for i in range(0, data.size() - 2, 2):
		var curr_pos: Vector3 = data[i] - global_transform.origin
		var curr_tangent: Vector3 = data[i + 1]
		var curr_normal: Vector3 = (data[i] - camera_position).normalized()
		var curr_binorm: Vector3 = curr_normal.cross(curr_tangent).normalized()

		var next_pos: Vector3 = data[i + 2] - global_transform.origin
		var next_tangent: Vector3 = data[i + 3]
		var next_normal: Vector3 = (data[i + 2] - camera_position).normalized()
		var next_binorm: Vector3 = next_normal.cross(next_tangent).normalized()
	
		_draw_quad([
			curr_pos - curr_binorm * rope_width, 
			next_pos - next_binorm * rope_width, 
			next_pos + next_binorm * rope_width, 
			curr_pos + curr_binorm * rope_width], 
			-curr_binorm, tangent, 
			t, t + step,
			Color.black)
		t += step
	pass

func _draw_catmull_curve_baked() -> void:
	begin(Mesh.PRIMITIVE_TRIANGLES)
	
	# do drawing
	var camera = get_viewport().get_camera()
	var camera_position: Vector3 = camera.global_transform.origin if camera else Vector3.ZERO
	
	for i in range(0, simulation_particles - 1):
		var p0: Vector3 = particle_data.pos_curr[i] - particle_data.tangents[i] * get_segment_length() if i == 0 else particle_data.pos_curr[i - 1]
		var p1: Vector3 = particle_data.pos_curr[i]
		var p2: Vector3 = particle_data.pos_curr[i + 1]
		var p3: Vector3 = particle_data.pos_curr[i + 1] + particle_data.tangents[i + 1] * get_segment_length() if i == simulation_particles - 2 else particle_data.pos_curr[i + 2]
		
		var cam_dist_particle: Vector3 = camera_position - p1
		var interval_data: PoolVector3Array
		var rope_draw_subdivs: float = 1.0
		# dont subdivide if farther than subdiv_lod_distance units from camera
		if cam_dist_particle.length_squared() <= subdiv_lod_distance * subdiv_lod_distance:
			# subdivision based on dot product of segments
			var tangent_dots: float = particle_data.tangents[i].dot(particle_data.tangents[i + 1])
			if tangent_dots >= COS_5_DEG:
				# 0 subdivisions
				interval_data = catmull_interpolate_in_step_one(p0, p1, p2, p3)
				rope_draw_subdivs = 1.0
			elif tangent_dots >= COS_15_DEG:
				# 2 subdivisions
				interval_data = catmull_interpolate_in_step_halfs(p0, p1, p2, p3)
				rope_draw_subdivs = 0.5
			elif tangent_dots >= COS_30_DEG:
				# 3 subdivisions
				interval_data = catmull_interpolate_in_step_thirds(p0, p1, p2, p3)
				rope_draw_subdivs = 0.333333
			else:
				# 4 subdivisions
				interval_data = catmull_interpolate_in_step_fourths(p0, p1, p2, p3)
				rope_draw_subdivs = 0.25
		else:
			interval_data = catmull_interpolate_in_step_one(p0, p1, p2, p3)
		
		_draw_interval(interval_data, camera_position, rope_draw_subdivs, particle_data.tangents[i])
	end()

# unused func use catmull_curve_baked instead, it is faster
func _draw_catmull_curve() -> void:
	begin(Mesh.PRIMITIVE_TRIANGLES)
	
	# do drawing
	var camera = get_viewport().get_camera()
	var camera_position: Vector3 = camera.global_transform.origin if camera else Vector3.ZERO
	
	for i in range(0, simulation_particles - 1):
		var p0: Vector3 = particle_data.pos_curr[i] - particle_data.tangents[i] * get_segment_length() if i == 0 else particle_data.pos_curr[i - 1]
		var p1: Vector3 = particle_data.pos_curr[i]
		var p2: Vector3 = particle_data.pos_curr[i + 1]
		var p3: Vector3 = particle_data.pos_curr[i + 1] + particle_data.tangents[i + 1] * get_segment_length() if i == simulation_particles - 2 else particle_data.pos_curr[i + 2]
		
		var rope_draw_subdivs: float = 1.0
		var cam_dist_particle: Vector3 = camera_position - p1
		# dont subdivide if farther than subdiv_lod_distance units from camera
		if cam_dist_particle.length_squared() <= subdiv_lod_distance * subdiv_lod_distance:
			var tangent_dots: float = particle_data.tangents[i].dot(particle_data.tangents[i + 1])
			if tangent_dots >= COS_5_DEG:
				rope_draw_subdivs = 1.0 # 0 subdivisions
			elif tangent_dots >= COS_15_DEG:
				rope_draw_subdivs = 0.5 # 2 subdivisions
			elif tangent_dots >= COS_30_DEG:
				rope_draw_subdivs = 0.33333 # 3 subdivisions
			else:
				rope_draw_subdivs = 0.25 # 4 subdivisions
		
		var t = 0.0
		var step = rope_draw_subdivs
		while t <= 1.0:
			var point1_data: PoolVector3Array = catmull_interpolate(p0, p1, p2, p3, t)
			var point2_data: PoolVector3Array = catmull_interpolate(p0, p1, p2, p3, min(t + step, 1.0))
			
			var curr_pos: Vector3 = point1_data[0] - global_transform.origin
			var curr_tangent: Vector3 = point1_data[1]
			var curr_normal: Vector3 = (point1_data[0] - camera_position).normalized()
			var curr_binorm: Vector3 = curr_normal.cross(curr_tangent).normalized()

			var next_pos: Vector3 = point2_data[0] - global_transform.origin
			var next_tangent: Vector3 = point2_data[1]
			var next_normal: Vector3 = (point2_data[0] - camera_position).normalized()
			var next_binorm: Vector3 = next_normal.cross(next_tangent).normalized()

			_draw_quad([
				curr_pos - curr_binorm * rope_width, 
				next_pos - next_binorm * rope_width, 
				next_pos + next_binorm * rope_width, 
				curr_pos + curr_binorm * rope_width], 
				-curr_binorm, particle_data.tangents[i], 
				t, t + step,
				Color.black)
			t += step
	end()

func _calculate_rope_orientation_with_camera() -> void:
	var camera: Camera = get_viewport().get_camera()
	var camera_pos: Vector3 = camera.global_transform.origin if camera else Vector3.ZERO
	particle_data.tangents[0] = (particle_data.pos_curr[1] - particle_data.pos_curr[0]).normalized()
	particle_data.normals[0] = (particle_data.pos_curr[0] - camera_pos).normalized()
	particle_data.binormals[0] = particle_data.normals[0].cross(particle_data.tangents[0]).normalized()

	particle_data.tangents[-1] = (particle_data.pos_curr[-1] - particle_data.pos_curr[-2]).normalized()
	particle_data.normals[-1] = (particle_data.pos_curr[-1] - camera_pos).normalized()
	particle_data.binormals[-1] = particle_data.normals[-1].cross(particle_data.tangents[-1]).normalized()

	for i in range(1, simulation_particles - 1):
		particle_data.tangents[i] = (particle_data.pos_curr[i + 1] - particle_data.pos_curr[i - 1]).normalized()
		particle_data.normals[i] = (particle_data.pos_curr[i] - camera_pos).normalized()
		particle_data.binormals[i] = particle_data.normals[i].cross(particle_data.tangents[i]).normalized()

func _create_rope() -> void:
	var end_location: Vector3
	if not attach_end_to.is_empty():
		end_location = get_node(attach_end_to).global_transform.origin
	else:
		end_location = global_transform.origin + Vector3.DOWN * rope_length
	
	var direction: Vector3 = (end_location - global_transform.origin).normalized()
	var gap = get_segment_length()
	
	if particle_data == null:
		particle_data = RopeParticleData.new()
	
	particle_data.resize(simulation_particles)
	# place particle_data initially in a line from start to end
	for i in range(simulation_particles):
		particle_data.pos_prev[i] = global_transform.origin + direction * gap * i
		particle_data.pos_curr[i] = particle_data.pos_prev[i]
		particle_data.accel[i] = gravity * gravity_scale
		particle_data.is_attached[i] = false
	
	particle_data.is_attached[0] = attach_start
	particle_data.is_attached[simulation_particles - 1] = not attach_end_to.is_empty()
	particle_data.pos_prev[simulation_particles - 1] = end_location
	particle_data.pos_curr[simulation_particles - 1] = end_location
	
	for _iter in range(preprocess_iterations):
		_verlet_process(1.0 / 60.0)
		# commenting this shaves quite a bit more startup time
		_apply_constraints()
	
	_calculate_rope_orientation_with_camera()

func _destroy_rope() -> void:
	particle_data.resize(0)
	simulation_particles = 0

func _draw_rope_particles() -> void:
	begin(Mesh.PRIMITIVE_LINES)
	for i in range(simulation_particles):
		var pos_curr: Vector3 = particle_data.pos_curr[i] - global_transform.origin
		var tangent: Vector3 = particle_data.tangents[i]
		var normal: Vector3 = particle_data.normals[i]
		var binormal: Vector3 = particle_data.binormals[i]
		
		#material_override.set("vertex_color_use_as_albedo", true)
		#set_color(Color.red)
		add_vertex(pos_curr)
		add_vertex(pos_curr + 0.3 * tangent)
		
		#set_color(Color.green)
		add_vertex(pos_curr)
		add_vertex(pos_curr + 0.3 * normal)
		
		#set_color(Color.blue)
		add_vertex(pos_curr)
		add_vertex(pos_curr + 0.3 * binormal)
		#material_override.set("vertex_color_use_as_albedo", false)
	end()

# give in clockwise order, or maybe anticlockwise?
func _draw_quad(vs: Array, n: Vector3, _t: Vector3, uvx0: float, uvx1: float, c: Color) -> void:
	set_color(c)
	set_normal(n)
	# for normal mapping???
	# set_tangent(Plane(-t, 0.0))
	set_uv(Vector2(uvx0, 0.0))
	add_vertex(vs[0])
	set_uv(Vector2(uvx1, 0.0))
	add_vertex(vs[1])
	set_uv(Vector2(uvx1, 1.0))
	add_vertex(vs[2])
	set_uv(Vector2(uvx0, 0.0))
	add_vertex(vs[0])
	set_uv(Vector2(uvx1, 1.0))
	add_vertex(vs[2])
	set_uv(Vector2(uvx0, 1.0))
	add_vertex(vs[3])

func _apply_forces() -> void:
	for i in range(simulation_particles):
		var total_accel: Vector3 = Vector3.ZERO
		if apply_gravity:
			total_accel += gravity * gravity_scale
		if apply_wind and wind_noise != null:
			var pos: Vector3 = particle_data.pos_curr[i]
			var wind_force: float = wind_noise.get_noise_4d(pos.x, pos.y, pos.z, time)
			total_accel += wind_scale * wind * wind_force
		if apply_damping:
			var velocity: Vector3 = particle_data.pos_curr[i] - particle_data.pos_prev[i]
			var drag = -damping_factor * velocity.length() * velocity
			total_accel += drag
		
		particle_data.accel[i] = total_accel

func _verlet_process(delta: float) -> void:
	for i in range(simulation_particles):
		if not particle_data.is_attached[i]:
			var pos_curr: Vector3 = particle_data.pos_curr[i]
			var pos_prev: Vector3 = particle_data.pos_prev[i]
			particle_data.pos_curr[i] = 2.0 * pos_curr - pos_prev + delta * delta * particle_data.accel[i]
			particle_data.pos_prev[i] = pos_curr

var prev_normal: Vector3 # needed for _apply_constraints
func _apply_constraints() -> void:
	for _n in range(iterations):
		for i in range(simulation_particles - 1):
			var r: Vector3 = particle_data.pos_curr[i + 1] - particle_data.pos_curr[i]
			var d: float = r.length() - get_segment_length()
			r = r.normalized()
			if particle_data.is_attached[i]:
				particle_data.pos_curr[i + 1] -= r * d * stiffness
			elif particle_data.is_attached[i + 1]:
				particle_data.pos_curr[i] += r * d * stiffness
			else:
				particle_data.pos_curr[i] += r * d * 0.5 * stiffness
				particle_data.pos_curr[i + 1] -= r * d * 0.5 * stiffness
	
	if apply_collision:
		# check if any objects are in its aabb before doing collisions
		var aabb: AABB = get_aabb()
		collision_check_box.extents = aabb.size * 0.5
		collision_check_param.transform.origin = global_transform.origin + aabb.position + aabb.size * 0.5
		var colliders: Array = space_state.intersect_shape(collision_check_param, 1)
		if len(colliders) >= 1:
			for i in range(simulation_particles - 1):
				var result: Dictionary = space_state.intersect_ray(particle_data.pos_curr[i] + prev_normal * 0.4, particle_data.pos_curr[i + 1], [], collision_mask)
				if result:
					prev_normal = result.normal
					var ydiff: Vector3 = result.position - particle_data.pos_curr[i + 1]
					ydiff = ydiff.project(result.normal)
					#ydiff += rope_width * 0.5 * result.normal
					particle_data.pos_curr[i + 1] += ydiff
					particle_data.pos_prev[i + 1] = particle_data.pos_curr[i + 1]

func _get_configuration_warning() -> String:
	visibility_notifier = null
	for c in get_children():
		if c is VisibilityNotifier:
			visibility_notifier = c
			break
	if visibility_notifier == null:
		return "Consider adding a VisibilityNotifier as a child for performance (it's bounds is automatically set at runtime)"
	return ""

func _ready() -> void:
	for c in get_children():
		if c is VisibilityNotifier:
			visibility_notifier = c
			var _err := visibility_notifier.connect("camera_entered", self, "_on_VisibilityNotifier_camera_entered")
			_err = visibility_notifier.connect("camera_exited", self, "_on_VisibilityNotifier_camera_exited")
			break
	
	# Configure collision box
	var aabb: AABB = get_aabb()
	collision_check_box = BoxShape.new()
	collision_check_box.extents = aabb.size * 0.5
	
	# configure collision check param
	collision_check_param = PhysicsShapeQueryParameters.new()
	collision_check_param.collision_mask = collision_mask
	collision_check_param.margin = 0.1
	collision_check_param.shape_rid = collision_check_box.get_rid()
	collision_check_param.transform.origin = aabb.position + collision_check_box.extents
	
	_create_rope()

func _physics_process(delta: float) -> void:
	if Engine.editor_hint and (particle_data == null or particle_data.is_empty()):
		_create_rope()
	
	time += delta
	
	# warning-ignore:integer_division
	if Engine.get_physics_frames() % int(Engine.iterations_per_second / simulation_rate) != 0:
		return
		
	# make the end follow the end attached object or stay at its attached location
	if not attach_end_to.is_empty():
		particle_data.pos_curr[-1] = get_node(attach_end_to).global_transform.origin
	if attach_start:
		particle_data.pos_curr[0] = global_transform.origin

	if simulate:
		_apply_forces()
	# warning-ignore:integer_division
		_verlet_process(delta * float(Engine.iterations_per_second / simulation_rate))
		_apply_constraints()
	
	if visibility_notifier != null:
		visibility_notifier.aabb = get_aabb()

	# drawing
	if draw:
		_calculate_rope_orientation_with_camera()
		# @HACK: rope doesnt draw from origin to attach_end_to correctly if rotated
		# calling to_local() in the drawing code is too slow
		global_transform.basis = Basis.IDENTITY
		clear()
		_draw_catmull_curve_baked()
		#_draw_catmull_curve()
		#_draw_linear_curve()
		#_draw_rope_particles()

func _on_VisibilityNotifier_camera_exited(_camera: Camera) -> void:
	#simulate = false
	draw = false

func _on_VisibilityNotifier_camera_entered(_camera: Camera) -> void:
	#simulate = true
	draw = true
