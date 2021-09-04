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
