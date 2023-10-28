using Godot;
using System;
using System.Collections.Generic;
using System.Linq;

namespace VerletRope4;

/*
Rewritten to be used with Godot 4.0+.
No additional license applied, feel free to use without my notice, though do not forget to still apply original license.
(c) Timofey Ivanov / tshmofen

MIT License

Copyright (c) 2023 Zae Chao(zaevi)
Copyright (c) 2021 Shashank C

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

[Tool]
[GlobalClass]
public partial class VerletRope : MeshInstance3D
{
    private struct RopeParticleData
    {
        public Vector3 PositionPrevious { get; set; }
        public Vector3 PositionCurrent { get; set; }
        public Vector3 Acceleration { get; set; }
        public bool IsAttached { get; set; }
        public Vector3 Tangent { get; set; }
        public Vector3 Normal { get; set; }
        public Vector3 Binormal { get; set; }
    }

    private const string DefaultMaterialPath = "res://addons/verlet_rope_4/materials/rope_default.material";
    private const string NoNotifierWarning = "Consider adding a VisibleOnScreenNotifier3D as a child for performance (it's bounds is automatically set at runtime)";
    private const string NormalParameter = "normal";
    private const string PositionParameter = "position";

    private static readonly float Cos5Deg = Mathf.Cos(Mathf.DegToRad(5.0f));
    private static readonly float Cos15Deg = Mathf.Cos(Mathf.DegToRad(15.0f));
    private static readonly float Cos30Deg = Mathf.Cos(Mathf.DegToRad(30.0f));

    private double _time;
    private Vector3 _prevNormal;
    private RopeParticleData[] _particleData;
    private VisibleOnScreenNotifier3D _ropeVisibleOnScreenNotifier3D;

    private ImmediateMesh _mesh;
    private BoxShape3D _collisionCheckBox;
    private PhysicsDirectSpaceState3D _spaceState;
    private PhysicsShapeQueryParameters3D _collisionCheckParam;

    private Node3D _attachEnd;
    private bool _attachStart = true;
    private int _simulationParticles = 10;

    [ExportGroup("Basics")]
    [Export]
    public bool AttachStart
    {
        set
        {
            _attachStart = value;
            if (_particleData != null)
            {
                _particleData[0].IsAttached = value;
            }
        }
        get => _attachStart;
    }
    [Export]
    public Node3D AttachEnd
    {
        set
        {
            _attachEnd = value;

            if (_particleData != null)
            {
                _particleData[^1].IsAttached = _attachEnd != null;
            }
        }
        get => _attachEnd;
    }
    [Export] public float RopeLength { get; set; } = 5.0f;
    [Export] public float RopeWidth { get; set; } = 0.07f;
    [Export(PropertyHint.Range, "3,300")]
    public int SimulationParticles
    {
        set
        {
            _simulationParticles = value;

            if (_particleData == null)
            {
                return;
            }

            Array.Resize(ref _particleData, _simulationParticles);
            CreateRope();
        }
        get => _simulationParticles;
    }

    [ExportGroup("Simulation")]
    [Export] public int Iterations { get; set; } = 2;
    [Export] public int PreprocessIterations { get; set; } = 5;
    [Export(PropertyHint.Range, "0.0, 1.5")] public float Stiffness { get; set; } = 0.9f;
    [Export] public bool Simulate { get; set; } = true;
    [Export] public bool Draw { get; set; } = true;
    [Export] public bool StartDrawSimulationOnStart { get; set; } = true;
    [Export] public float SubdivisionLodDistance { get; set; } = 15.0f;

    [ExportGroup("Gravity")]
    [Export] public bool ApplyGravity { get; set; } = true;
    [Export] public Vector3 Gravity { get; set; } = Vector3.Down * 9.8f;
    [Export] public float GravityScale { get; set; } = 1.0f;

    [ExportGroup("Wind")]
    [Export] public bool ApplyWind { get; set; } = false;
    [Export] public FastNoiseLite WindNoise { get; set; } = null;
    [Export] public Vector3 Wind { get; set; } = new(1.0f, 0.0f, 0.0f);
    [Export] public float WindScale { get; set; } = 20.0f;

    [ExportGroup("Damping")]
    [Export] public bool ApplyDamping { get; set; } = true;
    [Export] public float DampingFactor { get; set; } = 100.0f;

    [ExportGroup("Collision")]
    [Export] public bool ApplyCollision { get; set; } = false;
    [Export(PropertyHint.Layers3DPhysics)] public uint CollisionMask { get; set; } = 1;

    [ExportGroup("Debug")]
    [Export] public bool DrawDebugParticles { get; set; } = false;

    #region Util

    private float GetSegmentLength()
    {
        return RopeLength / (SimulationParticles - 1);
    }

    private void AddParticleAtEnd(bool adjustLength)
    {
        var p = new RopeParticleData();
        ref var last = ref _particleData[^1];

        p.PositionPrevious = last.PositionPrevious + Vector3.Back * 0.01f;
        p.PositionCurrent = last.PositionCurrent + Vector3.Back * 0.01f;
        p.Acceleration = last.Acceleration;
        p.IsAttached = last.IsAttached;
        p.Tangent = last.Tangent;
        p.Normal = last.Normal;
        p.Binormal = last.Binormal;

        last.IsAttached = false;

        SimulationParticles += 1;
        Array.Resize(ref _particleData, SimulationParticles);
        _particleData[SimulationParticles - 1] = p;

        if (adjustLength)
        {
            RopeLength += (RopeLength / SimulationParticles);
        }
    }

    private static void CatmullInterpolate(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float tension, float t, out Vector3 point, out Vector3 tangent)
    {
        // this is fast catmull spline

        var tSqr = t * t;
        var tCube = tSqr * t;

        var m1 = (1.0f - tension) * 0.5f * (p2 - p0);
        var m2 = (1.0f - tension) * 0.5f * (p3 - p1);

        var a = 2.0f * (p1 - p2) + m1 + m2;
        var b = -3.0f * (p1 - p2) - 2.0f * m1 - m2;

        point = a * tCube + b * tSqr + m1 * t + p1;
        tangent = (3.0f * a * tSqr + 2.0f * b * t + m1).Normalized();
    }

    private void DrawCatmullCurve()
    {
        _mesh.SurfaceBegin(Mesh.PrimitiveType.Triangles);

        var camera = GetViewport().GetCamera3D();
        var camPos = camera?.GlobalPosition ?? Vector3.Zero;

        for (var i = 0; i < SimulationParticles - 1; ++i)
        {
            var p0 = i == 0 ? _particleData[i].PositionCurrent - _particleData[i].Tangent * GetSegmentLength() : _particleData[i - 1].PositionCurrent;
            var p1 = _particleData[i].PositionCurrent;
            var p2 = _particleData[i + 1].PositionCurrent;
            var p3 = i == SimulationParticles - 2 ? _particleData[i + 1].PositionCurrent + _particleData[i + 1].Tangent * GetSegmentLength() : _particleData[i + 2].PositionCurrent;

            var ropeDrawSubdivisions = 1.0f;
            var camDistParticle = camPos - p1;

            if (camDistParticle.LengthSquared() <= SubdivisionLodDistance * SubdivisionLodDistance)
            {
                var tangentDots = _particleData[i].Tangent.Dot(_particleData[i + 1].Tangent);
                ropeDrawSubdivisions =
                    tangentDots >= Cos5Deg ? 1.0f :
                    tangentDots >= Cos15Deg ? 0.5f :
                    tangentDots >= Cos30Deg ? 0.33333f : 0.25f;
            }

            var t = 0.0f;
            var step = ropeDrawSubdivisions;
            while (t <= 1.0f)
            {
                CatmullInterpolate(p0, p1, p2, p3, 0.0f, t, out var currentPosition, out var currentTangent);
                CatmullInterpolate(p0, p1, p2, p3, 0.0f, Mathf.Min(t + step, 1.0f), out var nextPosition, out var nextTangent);

                var currentNormal = (currentPosition - camPos).Normalized();
                var currentBinormal = currentNormal.Cross(currentTangent).Normalized();
                currentPosition -= GlobalPosition;

                var nextNormal = (nextPosition - camPos).Normalized();
                var nextBinormal = nextNormal.Cross(nextTangent).Normalized();
                nextPosition -= GlobalPosition;

                Vector3[] vs =
                {
                    currentPosition - currentBinormal * RopeWidth,
                    nextPosition - nextBinormal * RopeWidth,
                    nextPosition + nextBinormal * RopeWidth,
                    currentPosition + currentBinormal * RopeWidth
                };

                DrawQuad(vs, -currentBinormal, t, t + step);
                t += step;
            }
        }

        _mesh.SurfaceEnd();
    }

    private void CalculateRopeOrientationWithCamera()
    {
        var camera = GetViewport().GetCamera3D();
        var cameraPos = camera?.GlobalPosition ?? Vector3.Zero;

        ref var first = ref _particleData[0];
        first.Tangent = (_particleData[1].PositionCurrent - first.PositionCurrent).Normalized();
        first.Normal = (first.PositionCurrent - cameraPos).Normalized();
        first.Binormal = first.Normal.Cross(first.Tangent).Normalized();

        ref var last = ref _particleData[SimulationParticles - 1];
        last.Tangent = (last.PositionCurrent - _particleData[SimulationParticles - 2].PositionCurrent).Normalized();
        last.Normal = (last.PositionCurrent - cameraPos).Normalized();
        last.Binormal = last.Normal.Cross(last.Tangent).Normalized();

        for (var i = 1; i < SimulationParticles - 1; i++)
        {
            ref var it = ref _particleData[i];
            it.Tangent = (_particleData[i + 1].PositionCurrent - _particleData[i - 1].PositionCurrent).Normalized();
            it.Normal = (_particleData[i].PositionCurrent - cameraPos).Normalized();
            it.Binormal = _particleData[i].Normal.Cross(_particleData[i].Tangent).Normalized();
        }
    }

    private void DrawQuad(IReadOnlyList<Vector3> vs, Vector3 n, float uvx0, float uvx1)
    {
        // NOTE: still may need tangents setup for normal mapping, not tested
        // SetTangent(new Plane(-t, 0.0f));
        _mesh.SurfaceSetNormal(n);
        _mesh.SurfaceSetUV(new Vector2(uvx0, 0.0f));
        _mesh.SurfaceAddVertex(vs[0]);
        _mesh.SurfaceSetUV(new Vector2(uvx1, 0.0f));
        _mesh.SurfaceAddVertex(vs[1]);
        _mesh.SurfaceSetUV(new Vector2(uvx1, 1.0f));
        _mesh.SurfaceAddVertex(vs[2]);
        _mesh.SurfaceSetUV(new Vector2(uvx0, 0.0f));
        _mesh.SurfaceAddVertex(vs[0]);
        _mesh.SurfaceSetUV(new Vector2(uvx1, 1.0f));
        _mesh.SurfaceAddVertex(vs[2]);
        _mesh.SurfaceSetUV(new Vector2(uvx0, 1.0f));
        _mesh.SurfaceAddVertex(vs[3]);
    }

    private void ApplyForces()
    {
        for (var i = 0; i < SimulationParticles; ++i)
        {
            ref var p = ref _particleData[i];
            var totalAcceleration = Vector3.Zero;

            if (ApplyGravity)
            {
                totalAcceleration += Gravity * GravityScale;
            }

            if (ApplyWind && WindNoise != null)
            {
                var timedPosition = p.PositionCurrent + Vector3.One * (float)_time;
                var windForce = WindNoise.GetNoise3D(timedPosition.X, timedPosition.Y, timedPosition.Z);
                totalAcceleration += WindScale * Wind * windForce;
            }

            if (ApplyDamping)
            {
                var velocity = _particleData[i].PositionCurrent - _particleData[i].PositionPrevious;
                var drag = -DampingFactor * velocity.Length() * velocity;
                totalAcceleration += drag;
            }

            p.Acceleration = totalAcceleration;
        }
    }

    private void VerletProcess(float delta)
    {
        for (var i = 0; i < SimulationParticles; ++i)
        {
            ref var p = ref _particleData[i];

            if (p.IsAttached)
            {
                continue;
            }

            var positionCurrentCopy = p.PositionCurrent;
            p.PositionCurrent = 2.0f * p.PositionCurrent - p.PositionPrevious + delta * delta * p.Acceleration;
            p.PositionPrevious = positionCurrentCopy;
        }
    }

    private void ApplyConstraints()
    {
        for (var iteration = 0; iteration < Iterations; ++iteration)
        {
            for (var i = 0; i < SimulationParticles - 1; ++i)
            {
                var r = _particleData[i + 1].PositionCurrent - _particleData[i].PositionCurrent;
                var d = r.Length() - GetSegmentLength();
                r = r.Normalized();

                if (_particleData[i].IsAttached)
                {
                    _particleData[i + 1].PositionCurrent -= r * d * Stiffness;
                }
                else if (_particleData[i + 1].IsAttached)
                {
                    _particleData[i].PositionCurrent += r * d * Stiffness;
                }
                else
                {
                    _particleData[i].PositionCurrent += r * d * 0.5f * Stiffness;
                    _particleData[i + 1].PositionCurrent -= r * d * 0.5f * Stiffness;
                }
            }
        }

        if (!ApplyCollision)
        {
            return;
        }

        var visual = GetAabb();
        if (visual.Size == Vector3.Zero)
        {
            return;
        }

        _collisionCheckBox.Size = visual.Size;
        _collisionCheckParam.Transform = new Transform3D(_collisionCheckParam.Transform.Basis, GlobalPosition + visual.Position + visual.Size / 2);

        var collisions = _spaceState.IntersectShape(_collisionCheckParam, 1);
        if (collisions.Count < 1)
        {
            return;
        }

        for (var i = 0; i < SimulationParticles - 1; ++i)
        {
            var result = _spaceState.IntersectRay(new PhysicsRayQueryParameters3D
            {
                From = _particleData[i].PositionCurrent + _prevNormal * 0.4f,
                To = _particleData[i + 1].PositionCurrent,
                CollisionMask = CollisionMask
            });

            if (result.Count <= 0)
            {
                continue;
            }

            _prevNormal = result[NormalParameter].AsVector3();
            var yDifference = result[PositionParameter].AsVector3() - _particleData[i + 1].PositionCurrent;
            yDifference = yDifference.Project(_prevNormal);

            _particleData[i + 1].PositionCurrent += yDifference;
            _particleData[i + 1].PositionPrevious = _particleData[i + 1].PositionCurrent;
        }
    }

    #endregion

    public override string[] _GetConfigurationWarnings()
    {
        _ropeVisibleOnScreenNotifier3D = (VisibleOnScreenNotifier3D)GetChildren().FirstOrDefault(c => c is VisibleOnScreenNotifier3D);
        return _ropeVisibleOnScreenNotifier3D == null ? new[] { NoNotifierWarning } : Array.Empty<string>();
    }

    public override void _Ready()
    {
        if (!Engine.IsEditorHint() && StartDrawSimulationOnStart)
        {
            Draw = true;
            Simulate = true;
        }

        _mesh = Mesh as ImmediateMesh;
        if (_mesh == null)
        {
            Mesh = _mesh = new ImmediateMesh();
            _mesh.ResourceLocalToScene = true;
        }

        var notifier = (VisibleOnScreenNotifier3D)GetChildren().FirstOrDefault(c => c is VisibleOnScreenNotifier3D);
        if (notifier != null)
        {
            _ropeVisibleOnScreenNotifier3D = notifier;
            _ropeVisibleOnScreenNotifier3D.ScreenEntered += () => Draw = true;
            _ropeVisibleOnScreenNotifier3D.ScreenExited += () => Draw = false;
        }

        _spaceState = GetWorld3D().DirectSpaceState;
        var visuals = GetAabb();

        _collisionCheckBox = new BoxShape3D
        {
            Size = visuals.Size
        };

        _collisionCheckParam = new PhysicsShapeQueryParameters3D
        {
            ShapeRid = _collisionCheckBox.GetRid(),
            CollisionMask = CollisionMask,
            Margin = 0.1f
        };

        _collisionCheckParam.Transform = new Transform3D(_collisionCheckParam.Transform.Basis, GlobalPosition + visuals.Position + visuals.Size / 2);
        MaterialOverride ??= GD.Load<StandardMaterial3D>(DefaultMaterialPath);

        CreateRope();
    }

    public override void _PhysicsProcess(double delta)
    {
        if (Engine.IsEditorHint())
        {
            if (_particleData == null)
            {
                CreateRope();
            }
        }

        _time += delta;

        // make the end follow the end attached object or stay at its attached location
        if (_attachEnd != null)
        {
            _particleData![SimulationParticles - 1].PositionCurrent = _attachEnd.GlobalPosition;
        }

        if (AttachStart)
        {
            _particleData![0].PositionCurrent = GlobalPosition;
        }

        if (Simulate)
        {
            ApplyForces();
            VerletProcess((float)delta);
            ApplyConstraints();
        }

        if (!Draw)
        {
            return;
        }

        CalculateRopeOrientationWithCamera();
        _mesh.ClearSurfaces();
        // @HACK: rope doesn't draw from origin to attach_end_to correctly if rotated
        // calling to_local() in the drawing code is too slow
        GlobalTransform = new Transform3D(Basis.Identity, GlobalPosition);

        if (DrawDebugParticles)
        {
            DrawRopeDebugParticles();
        }

        DrawCatmullCurve();

        if (_ropeVisibleOnScreenNotifier3D != null)
        {
            _ropeVisibleOnScreenNotifier3D.Aabb = GetAabb();
        }
    }

    public void CreateRope()
    {
        var endLocation = GlobalPosition + Vector3.Down * RopeLength;

        if (_attachEnd != null)
        {
            endLocation = _attachEnd.GlobalPosition;
        }

        var direction = (endLocation - GlobalPosition).Normalized();
        var gap = GetSegmentLength();

        _particleData = new RopeParticleData[SimulationParticles];

        for (var i = 0; i < SimulationParticles; ++i)
        {
            _particleData[i] = new RopeParticleData();
            ref var p = ref _particleData[i];
            p.PositionPrevious = GlobalPosition + direction * gap * i;
            p.PositionCurrent = p.PositionPrevious;
            p.IsAttached = false;
            p.Acceleration = Gravity * GravityScale;
            p.Tangent = p.Normal = p.Binormal = Vector3.Zero;
        }

        ref var first = ref _particleData[0];
        ref var last = ref _particleData[SimulationParticles - 1];
        first.IsAttached = AttachStart;
        last.IsAttached = _attachEnd != null;
        last.PositionPrevious = endLocation;
        last.PositionCurrent = endLocation;

        for (var i = 0; i < PreprocessIterations; ++i)
        {
            VerletProcess(1.0f / (float)Engine.GetFramesPerSecond());
            ApplyConstraints();
        }

        CalculateRopeOrientationWithCamera();
    }

    public void DestroyRope()
    {
        Array.Resize(ref _particleData, 0);
        SimulationParticles = 0;
    }

    public void DrawRopeDebugParticles()
    {
        _mesh.SurfaceBegin(Mesh.PrimitiveType.Lines);

        for (var i = 0; i < SimulationParticles; ++i)
        {
            var positionCurrent = _particleData[i].PositionCurrent - GlobalPosition;
            var tangent = _particleData[i].Tangent;
            var normal = _particleData[i].Normal;
            var binormal = _particleData[i].Binormal;

            _mesh.SurfaceAddVertex(positionCurrent);
            _mesh.SurfaceAddVertex(positionCurrent + 0.3f * tangent);

            _mesh.SurfaceAddVertex(positionCurrent);
            _mesh.SurfaceAddVertex(positionCurrent + 0.3f * normal);

            _mesh.SurfaceAddVertex(positionCurrent);
            _mesh.SurfaceAddVertex(positionCurrent + 0.3f * binormal);
        }

        _mesh.SurfaceEnd();
    }
}
