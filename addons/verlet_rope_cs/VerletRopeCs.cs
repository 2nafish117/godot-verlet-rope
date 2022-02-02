using Godot;
using System;

namespace Fish {
	[Tool]
	public class VerletRopeCs : ImmediateGeometry
	{
	const float INV_SQRT_2 = 1.0f / Mathf.Sqrt2;
	const float COS_5_DEG = 0.9961946980917455f; // Mathf.Cos(Mathf.Deg2Rad(5.0f));
	const float COS_10_DEG = 0.984807753012208f; // Mathf.Cos(Mathf.Deg2Rad(10.0f));
	const float COS_15_DEG = 0.9659258262890683f; // Mathf.Cos(Mathf.Deg2Rad(15.0f));
	const float COS_20_DEG = 0.9396926207859084f; // Mathf.Cos(Mathf.Deg2Rad(20.0f));
	const float COS_25_DEG = 0.9063077870366499f; // Mathf.Cos(Mathf.Deg2Rad(25.0f));
	const float COS_30_DEG = 0.8660254037844387f; // Mathf.Cos(Mathf.Deg2Rad(30.0f));

	private bool _attachStart = true;
	[Export] public bool AttachStart {
		set {
			_attachStart = value;
			if(_particleData != null) {
				_particleData[0].IsAttached = value;
			}
		}
		get {
			return _attachStart;
		}
	}

	// use AttachEndTo to attahc or detach end
	public bool AttachEnd {
		get {
			return (_attachEndTo != null && ! _attachEndTo.IsEmpty());
		}
	}

	private NodePath _attachEndTo;
	[Export] public NodePath AttachEndTo {
		set {
			_attachEndTo = value; 
			if(_particleData != null) {
				_particleData[_particleData.Length - 1].IsAttached = AttachEnd;
			}
		}
		get {
			return _attachEndTo;
		}
	}

	public Vector3 EndLocation {
		get {
			return _particleData[_particleData.Length - 1].PosCurr;
		}
	}

	[Export] public float RopeLength = 5.0f;
	[Export] public float RopeWidth = 0.07f;

	// @TODO: setget for SimulationParticles
	private int simulationParticles = 10;
	[Export(PropertyHint.Range, "3,300")] public int SimulationParticles {
		set {
			simulationParticles = value;
			if(_particleData != null) {
				Array.Resize<RopeParticleData>(ref _particleData, simulationParticles);
				CreateRope();
			}
		}
		get {
			return simulationParticles;
		}
	}

	[Export] public int Iterations = 2;
	[Export] public int PreprocessIterations = 5;
	[Export] public int SimulationRate = 60;
	[Export(PropertyHint.Range, "0.0, 1.5")] public float Stiffness = 0.9f;
	[Export] public bool Simulate = true;
	[Export] public bool Draw = true;
	[Export] public float SubdivLodDistance = 15.0f;

	[Export] public bool ApplyGravity = true;
	[Export] public Vector3 Gravity = Vector3.Down * 9.8f;
	[Export] public float GravityScale = 1.0f;

	[Export] public bool ApplyWind = false;
	[Export] public OpenSimplexNoise WindNoise = null;
	[Export] public Vector3 Wind = new Vector3(1.0f, 0.0f, 0.0f);
	[Export] public float WindScale = 20.0f;

	[Export] public bool ApplyDamping = true;
	[Export] public float DampingFactor = 100.0f;

	[Export] public bool ApplyCollision = false;
	[Export(PropertyHint.Layers3dPhysics)] public uint CollisionMask = 1;

	private float _time = 0.0f;
	private RopeParticleData[] _particleData;
	private VisibilityNotifier _ropeVisibilityNotifier;

	private PhysicsDirectSpaceState _spaceState;
	private BoxShape _collisionCheckBox;
	private PhysicsShapeQueryParameters _collisionCheckParam;

	struct RopeParticleData {
		public Vector3 PosPrev {get; set;}
		public Vector3 PosCurr {get; set;}
		public Vector3 Accel {get; set;}
		public bool IsAttached {get; set;}
		public Vector3 Tangent {get; set;}
		public Vector3 Normal {get; set;}
		public Vector3 Binormal {get; set;}
	}

	public float GetSegmentLength() {
		return RopeLength / (SimulationParticles - 1);
	}

	public void AddParticleAtEnd(bool AdjustLength) {
		RopeParticleData p = new RopeParticleData();
		ref RopeParticleData last = ref _particleData[_particleData.Length - 1];
		p.PosPrev = last.PosPrev + Vector3.Back * 0.01f;
		p.PosCurr = last.PosCurr + Vector3.Back * 0.01f;
		p.Accel = last.Accel;
		p.IsAttached = last.IsAttached;
		p.Tangent = last.Tangent;
		p.Normal = last.Normal;
		p.Binormal = last.Binormal;

		last.IsAttached = false;

		SimulationParticles += 1;
		Array.Resize<RopeParticleData>(ref _particleData, SimulationParticles);
		_particleData[SimulationParticles - 1] = p;

		if(AdjustLength) {
			RopeLength += (RopeLength / SimulationParticles);
		}
	}

	// unused func, draws simple quads from one particle to next
	private void DrawLinearCurve() {
		Begin(Mesh.PrimitiveType.Triangles);
		for(int i = 0;i < SimulationParticles - 1; ++i) {
			Vector3 currPos = _particleData[i].PosCurr - GlobalTransform.origin;
			Vector3 currBinorm = _particleData[i].Binormal;
			
			Vector3 nextPos = _particleData[i + 1].PosCurr - GlobalTransform.origin;
			Vector3 nextBinorm = _particleData[i + 1].Binormal;
			
			Vector3[] vs = {
				currPos - currBinorm * RopeWidth, 
				nextPos - nextBinorm * RopeWidth, 
				nextPos + nextBinorm * RopeWidth, 
				currPos + currBinorm * RopeWidth
			};

			DrawQuad(vs, -currBinorm, _particleData[i].Tangent, 0.0f, 1.0f, new Color(1.0f, 1.0f, 1.0f));
		}
		End();
	}

	// fast catmull spline
	private static void CatmullInterpolate(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float tension, float t, out Vector3 point, out Vector3 tangent) {
		float tSqr = t * t;
		float tCube = tSqr * t;

		Vector3 m1 = (1.0f - tension) * 0.5f * (p2 - p0);
		Vector3 m2 = (1.0f - tension) * 0.5f * (p3 - p1);

		Vector3 a = 2.0f * (p1 - p2) + m1 + m2;
		Vector3 b = -3.0f * (p1 - p2) - 2.0f * m1 - m2;
		Vector3 c = m1;
		Vector3 d = p1;
		point = a * tCube + b * tSqr + c * t + d;
		tangent = (3.0f * a * tSqr + 2.0f * b * t + c).Normalized();
	}

	private void DrawCatmullCurve() {
		Begin(Mesh.PrimitiveType.Triangles);
		
		// do drawing
		Camera camera = GetViewport().GetCamera();
		Vector3 camPos = camera != null ? camera.GlobalTransform.origin : Vector3.Zero;

		for(int i = 0;i < SimulationParticles - 1; ++i) {

			var p0 = i == 0 ? _particleData[i].PosCurr - _particleData[i].Tangent * GetSegmentLength() : _particleData[i - 1].PosCurr;
			var p1 = _particleData[i].PosCurr;
			var p2 = _particleData[i + 1].PosCurr;
			var p3 = i == SimulationParticles - 2 ? _particleData[i + 1].PosCurr + _particleData[i + 1].Tangent * GetSegmentLength() : _particleData[i + 2].PosCurr;
			
			float ropeDrawSubdivs = 1.0f;
			var camDistParticle = camPos - p1;
			// dont subdivide if farther than SubdivLodDistance units from camera
			if(camDistParticle.LengthSquared() <= SubdivLodDistance * SubdivLodDistance) {
				float tangentDots = _particleData[i].Tangent.Dot(_particleData[i + 1].Tangent);
				if(tangentDots >= COS_5_DEG) {
					ropeDrawSubdivs = 1.0f; // 0 subdivisions
				} else if(tangentDots >= COS_15_DEG) {
					ropeDrawSubdivs = 0.5f; // 2 subdivisions
				} else if(tangentDots >= COS_30_DEG) {
					ropeDrawSubdivs = 0.33333f; // 3 subdivisions
				} else {
					ropeDrawSubdivs = 0.25f; // 4 subdivisions
				}
			}
			
			float t = 0.0f;
			float step = ropeDrawSubdivs;
			while(t <= 1.0f) {
				Vector3 currPos;
				Vector3 currTangent;
				CatmullInterpolate(p0, p1, p2, p3, 0.0f, t, out currPos, out currTangent);
				Vector3 nextPos;
				Vector3 nextTangent;
				CatmullInterpolate(p0, p1, p2, p3, 0.0f, Mathf.Min(t + step, 1.0f), out nextPos, out nextTangent);

				Vector3 currNormal = (currPos - camPos).Normalized();
				Vector3 currBinorm = currNormal.Cross(currTangent).Normalized();
				currPos = currPos - GlobalTransform.origin;

				Vector3 nextNormal = (nextPos - camPos).Normalized();
				Vector3 nextBinorm = nextNormal.Cross(nextTangent).Normalized();
				nextPos = nextPos - GlobalTransform.origin;

				Vector3[] vs = {
					currPos - currBinorm * RopeWidth, 
					nextPos - nextBinorm * RopeWidth, 
					nextPos + nextBinorm * RopeWidth, 
					currPos + currBinorm * RopeWidth
				};
				
				DrawQuad(vs, -currBinorm, _particleData[i].Tangent, t, t + step, new Color(1.0f, 1.0f, 1.0f));
				t += step;
			}
		}
		End();
	}	

	private void CalculateRopeOrientationWithCamera() {
		Camera camera = GetViewport().GetCamera();
		Vector3 cameraPos = camera != null ? camera.GlobalTransform.origin : Vector3.Zero;
		
		ref RopeParticleData first = ref _particleData[0];
		first.Tangent = (_particleData[1].PosCurr - first.PosCurr).Normalized();
		first.Normal = (first.PosCurr - cameraPos).Normalized();
		first.Binormal = first.Normal.Cross(first.Tangent).Normalized();

		ref RopeParticleData last = ref _particleData[SimulationParticles - 1];
		last.Tangent = (last.PosCurr - _particleData[SimulationParticles - 2].PosCurr).Normalized();
		last.Normal = (last.PosCurr - cameraPos).Normalized();
		last.Binormal = last.Normal.Cross(last.Tangent).Normalized();

		for(int i = 1;i < SimulationParticles - 1; i++) {
			ref RopeParticleData it = ref _particleData[i];
			it.Tangent = (_particleData[i + 1].PosCurr - _particleData[i - 1].PosCurr).Normalized();
			it.Normal = (_particleData[i].PosCurr - cameraPos).Normalized();
			it.Binormal = _particleData[i].Normal.Cross(_particleData[i].Tangent).Normalized();
		}
	}

	private void CreateRope() {
		Vector3 endLocation = GlobalTransform.origin + Vector3.Down * RopeLength;
		if(AttachEnd) {
			endLocation = GetNode<Spatial>(_attachEndTo).GlobalTransform.origin;
		}
		
		Vector3 direction = (endLocation - GlobalTransform.origin).Normalized();
		float gap = GetSegmentLength();
		
		_particleData = new RopeParticleData[SimulationParticles];

		for(int i = 0; i < SimulationParticles; ++i) {
			_particleData[i] = new RopeParticleData();
			ref RopeParticleData p = ref _particleData[i];
			p.PosPrev = GlobalTransform.origin + direction * gap * i;
			p.PosCurr = p.PosPrev;
			p.IsAttached = false;
			p.Accel = Gravity * GravityScale;
			p.Tangent = p.Normal = p.Binormal = Vector3.Zero;
		}
		
		ref var first = ref _particleData[0];
		ref var last = ref _particleData[SimulationParticles - 1];
		first.IsAttached = AttachStart;
		last.IsAttached = AttachEnd;
		last.PosPrev = endLocation;
		last.PosCurr = endLocation;
		
		for(int i = 0;i < PreprocessIterations; ++i) {
			VerletProcess(1.0f / (float) Engine.IterationsPerSecond);
			// commenting this shaves quite a bit more startup time
			ApplyConstraints();

		}
		
		CalculateRopeOrientationWithCamera();
	}

	// unused func, maybe useful in code?
	private void DestroyRope() {
		Array.Resize<RopeParticleData>(ref _particleData, 0);
		SimulationParticles = 0;
	}

	// unused func, for debugging only
	private void DrawRopeParticles() {
		Begin(Mesh.PrimitiveType.Lines);
		for(int i = 0;i < SimulationParticles; ++i) {
			Vector3 posCurr = _particleData[i].PosCurr - GlobalTransform.origin;
			Vector3 tangent = _particleData[i].Tangent;
			Vector3 normal = _particleData[i].Normal;
			Vector3 binormal = _particleData[i].Binormal;
			
			// material_override.set("vertex_color_use_as_albedo", true)
			// set_color(Color.red)
			AddVertex(posCurr);
			AddVertex(posCurr + 0.3f * tangent);
			
			// set_color(Color.green)
			AddVertex(posCurr);
			AddVertex(posCurr + 0.3f * normal);
			
			// set_color(Color.blue)
			AddVertex(posCurr);
			AddVertex(posCurr + 0.3f * binormal);
			// material_override.set("vertex_color_use_as_albedo", false)
		}
		End();
	}

	// give in clockwise order, or maybe anticlockwise?
	private void DrawQuad(Vector3[] vs, Vector3 n, Vector3 t, float uvx0, float uvx1, Color c) {
		// func _draw_quad(vs: Array, n: Vector3, t: Vector3, c: Color) -> void:
		// SetColor(c);
		SetNormal(n);
		// for Normal mapping???
		// SetTangent(new Plane(-t, 0.0f));
		SetUv(new Vector2(uvx0, 0.0f));
		AddVertex(vs[0]);
		SetUv(new Vector2(uvx1, 0.0f));
		AddVertex(vs[1]);
		SetUv(new Vector2(uvx1, 1.0f));
		AddVertex(vs[2]);
		SetUv(new Vector2(uvx0, 0.0f));
		AddVertex(vs[0]);
		SetUv(new Vector2(uvx1, 1.0f));
		AddVertex(vs[2]);
		SetUv(new Vector2(uvx0, 1.0f));
		AddVertex(vs[3]);
	}

	private void ApplyForces() {
		for(int i = 0; i < SimulationParticles; ++i) {
			ref var p = ref _particleData[i];
			var totalAccel = Vector3.Zero;
			if(ApplyGravity) {
				totalAccel += Gravity * GravityScale;
			}
			if(ApplyWind && WindNoise != null) {
				float windForce = WindNoise.GetNoise4d(p.PosCurr.x, p.PosCurr.y, p.PosCurr.z, _time);
				totalAccel += WindScale * Wind * windForce;
			}
			if(ApplyDamping) {
				var velocity = _particleData[i].PosCurr - _particleData[i].PosPrev;
				var drag = -DampingFactor * velocity.Length() * velocity;
				totalAccel += drag;
			}
			p.Accel = totalAccel;
		}
	}

	private void VerletProcess(float delta) {
		for(int i = 0;i < SimulationParticles; ++i) {
			ref var p = ref _particleData[i];
			if(! p.IsAttached) {
				var posCurrCopy = p.PosCurr;
				p.PosCurr = 2.0f * p.PosCurr - p.PosPrev + delta * delta * p.Accel;
				p.PosPrev = posCurrCopy;
			}
		}
	}

	private Vector3 _prevNormal;
	private void ApplyConstraints() {
		for(int _iter = 0; _iter < Iterations; ++_iter) {
			for(int i = 0; i < SimulationParticles - 1; ++i) {
				Vector3 r = _particleData[i + 1].PosCurr - _particleData[i].PosCurr;

				float d = r.Length() - GetSegmentLength();
				r = r.Normalized();
				if(_particleData[i].IsAttached) {
					_particleData[i + 1].PosCurr -= r * d * Stiffness;
				} else if(_particleData[i + 1].IsAttached) {
					_particleData[i].PosCurr += r * d * Stiffness;
				} else {
					_particleData[i].PosCurr += r * d * 0.5f * Stiffness;
					_particleData[i + 1].PosCurr -= r * d * 0.5f * Stiffness;
				}
			}
		}

		if(ApplyCollision) {
			// Check if any colliders are inside ropes AABB before doing collisions
			_collisionCheckBox.Extents = GetAabb().Size * 0.5f;
			_collisionCheckParam.Transform = new Transform(_collisionCheckParam.Transform.basis, GlobalTransform.origin + GetAabb().Position + GetAabb().Size * 0.5f);
			var colliders = _spaceState.IntersectShape(_collisionCheckParam, 1);
			if(colliders.Count >= 1) {
				for(int i = 0;i < SimulationParticles - 1; ++i) {
					Godot.Collections.Dictionary result = _spaceState.IntersectRay(_particleData[i].PosCurr + _prevNormal * 0.4f, _particleData[i + 1].PosCurr, null, CollisionMask);
					if(result.Count > 0) {
						_prevNormal = (Vector3) result["normal"];
						Vector3 ydiff = (Vector3) result["position"] - _particleData[i + 1].PosCurr;
						ydiff = ydiff.Project((Vector3) result["normal"]);
						// ydiff += RopeWidth * 0.5f * result.normal;
						_particleData[i + 1].PosCurr += ydiff;
						_particleData[i + 1].PosPrev = _particleData[i + 1].PosCurr;
					}
				}
			}
		}
	}

	public override string _GetConfigurationWarning() {
		_ropeVisibilityNotifier = null;
		foreach(var c in GetChildren()) {
			if(c is VisibilityNotifier) {
				_ropeVisibilityNotifier = (VisibilityNotifier) c;
				break;
			}
		}
		if(_ropeVisibilityNotifier == null) {
			return "Consider adding a VisibilityNotifier as a child for performance (it's bounds is automatically set at runtime)";
		}
		return "";
	}

	public override void _Ready() {
		foreach(var c in GetChildren()) {
			if(c is VisibilityNotifier) {
				_ropeVisibilityNotifier = (VisibilityNotifier) c;
				_ropeVisibilityNotifier.Connect("camera_entered", this, "_on_VisibilityNotifier_camera_entered");
				_ropeVisibilityNotifier.Connect("camera_exited", this, "_on_VisibilityNotifier_camera_exited");
				break;
			}
		}
		_spaceState = GetWorld().DirectSpaceState;

		// Configure box
		_collisionCheckBox = new BoxShape();
		_collisionCheckBox.Extents = GetAabb().Size * 0.5f;
		// Configure physics shape query params
		_collisionCheckParam = new PhysicsShapeQueryParameters();
		_collisionCheckParam.CollisionMask = CollisionMask;
		_collisionCheckParam.Margin = 0.1f;
		_collisionCheckParam.ShapeRid = _collisionCheckBox.GetRid();
		_collisionCheckParam.Transform = new Transform(_collisionCheckParam.Transform.basis, GetAabb().Position + _collisionCheckBox.Extents);
		if(MaterialOverride == null) {
			MaterialOverride = GD.Load<SpatialMaterial>("res://addons/verlet_rope_cs/DefaultRope.material");
		}
		CreateRope();
	}

	public override void _PhysicsProcess(float delta) {
		if(Engine.EditorHint) {
			if(_particleData == null) {
				CreateRope();
			}
		}

		_time += delta;
		if(Engine.GetPhysicsFrames() % (ulong) (Engine.IterationsPerSecond / SimulationRate) != 0) {
			return;
		}

		// make the end follow the end attached object or stay at its attached location
		if(AttachEnd) {
			_particleData[SimulationParticles - 1].PosCurr = GetNode<Spatial>(_attachEndTo).GlobalTransform.origin;
		}
		if(AttachStart) {
			_particleData[0].PosCurr = GlobalTransform.origin;
		}

		if(Simulate) {
			ApplyForces();
			VerletProcess(delta * (float)(Engine.IterationsPerSecond / SimulationRate));
			ApplyConstraints();
		}

		if(_ropeVisibilityNotifier != null) {
			_ropeVisibilityNotifier.Aabb = GetAabb();
		}
		// drawing
		if(Draw) {
			CalculateRopeOrientationWithCamera();
			Clear();
			// @HACK: rope doesnt draw from origin to attach_end_to correctly if rotated
			// calling to_local() in the drawing code is too slow
			GlobalTransform = new Transform(Basis.Identity, GlobalTransform.origin);
			// DrawLinearCurve();
			// DrawRopeParticles();
			DrawCatmullCurve();
		}
	}

	private void _on_VisibilityNotifier_camera_exited(Camera c) {
		// Simulate = false;
		Draw = false;
	}

	private void _on_VisibilityNotifier_camera_entered(Camera c) {
		// Simulate = true;
		Draw = true;
	}

	/*End VerletRope.cs*/
	}
}