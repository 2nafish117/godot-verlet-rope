using Godot;
using System;
using System.Collections.Generic;
using System.Linq;

namespace VerletRope4;

/*
MIT License

Rewritten to be used with Godot 4.0+.
No additional license applied, feel free to use without my notice, though do not forget to still apply original license.
(c) Timofey Ivanov / tshmofen

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
public partial class VerletRope : MeshInstance3D
{
    #region Signals

    [Signal] public delegate void SimulationStepEventHandler(double delta);

    #endregion

    #region Variables

    #region Vars Private

    private const string DefaultMaterialPath = "res://addons/verlet_rope_4/materials/rope_default.material";
    private const string NoNotifierWarning = "Consider adding a VisibleOnScreenNotifier3D as a child for performance (it's bounds is automatically set at runtime)";
    private const string CreationStampMeta = "creation_stamp";
    private const string PositionParameter = "position";
    private const string ParticlesRangeHint = "3,300";
    private const string SimulationRangeHint = "30,265";
    private const string NormalParameter = "normal";
    private const float CollisionCheckLength = 0.4f;

    private static readonly float Cos5Deg = Mathf.Cos(Mathf.DegToRad(5.0f));
    private static readonly float Cos15Deg = Mathf.Cos(Mathf.DegToRad(15.0f));
    private static readonly float Cos30Deg = Mathf.Cos(Mathf.DegToRad(30.0f));

    private double _time;
    private Camera3D _camera;
    private double _simulationDelta;
    private Vector3 _previousNormal;
    private RopeParticleData _particleData;
    private VisibleOnScreenNotifier3D _visibleNotifier;

    private ImmediateMesh _mesh;
    private BoxShape3D _collisionCheckBox;
    private PhysicsDirectSpaceState3D _spaceState;
    private PhysicsShapeQueryParameters3D _collisionCheckParameters;

    private Node3D _attachEnd;
    private bool _attachStart = true;
    private int _simulationParticles = 10;

    #endregion

    #region Vars Basics

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

    [Export] public Node3D AttachEnd
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

    [Export(PropertyHint.Range, ParticlesRangeHint)]
    public int SimulationParticles
    {
        set
        {
            _simulationParticles = value;

            if (_particleData == null)
            {
                return;
            }

            _particleData.Resize(_simulationParticles);
            CreateRope();
        }
        get => _simulationParticles;
    }

    #endregion

    #region Vars Simulation

    [ExportGroup("Simulation")]
    [Export(PropertyHint.Range, SimulationRangeHint)] public int SimulationRate { get; set; } = 60;
    [Export] public int Iterations { get; set; } = 2;
    [Export] public int PreprocessIterations { get; set; } = 5;
    [Export] public float PreprocessDelta { get; set; } = 0.016f; 
    [Export(PropertyHint.Range, "0.0, 1.5")] public float Stiffness { get; set; } = 0.9f;
    [Export] public bool Simulate { get; set; } = true;
    [Export] public bool Draw { get; set; } = true;
    [Export] public bool StartDrawSimulationOnStart { get; set; } = true;
    [Export] public float SubdivisionLodDistance { get; set; } = 15.0f;

    #endregion

    #region Vars Gravity

    [ExportGroup("Gravity")]
    [Export] public bool ApplyGravity { get; set; } = true;
    [Export] public Vector3 Gravity { get; set; } = Vector3.Down * 9.8f;
    [Export] public float GravityScale { get; set; } = 1.0f;

    #endregion

    #region Vars Wind

    [ExportGroup("Wind")]
    [Export] public bool ApplyWind { get; set; } = false;
    [Export] public FastNoiseLite WindNoise { get; set; } = null;
    [Export] public Vector3 Wind { get; set; } = new(1.0f, 0.0f, 0.0f);
    [Export] public float WindScale { get; set; } = 20.0f;

    #endregion

    #region Vars Damping

    [ExportGroup("Damping")]
    [Export] public bool ApplyDamping { get; set; } = true;
    [Export] public float DampingFactor { get; set; } = 100.0f;

    #endregion

    #region Vars Collision

    [ExportGroup("Collision")]
    [Export] public bool ApplyCollision { get; set; } = false;
    [Export(PropertyHint.Layers3DPhysics)] public uint CollisionMask { get; set; } = 1;

    #endregion

    #region Vars Debug

    [ExportGroup("Debug")]
    [Export] public bool DrawDebugParticles { get; set; } = false;

    #endregion

    #endregion

    #region Internal Logic

    #region Util

    private (Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3) GetSimulationParticles(int index)
    {
        var p0 = (index == 0)
            ? _particleData[index].PositionCurrent - _particleData[index].Tangent * GetSegmentLength()
            : _particleData[index - 1].PositionCurrent;

        var p1 = _particleData[index].PositionCurrent;

        var p2 = _particleData[index + 1].PositionCurrent;

        var p3 = index == SimulationParticles - 2
            ? _particleData[index + 1].PositionCurrent + _particleData[index + 1].Tangent * GetSegmentLength()
            : _particleData[index + 2].PositionCurrent;

        return (p0, p1, p2, p3);
    }

    private VisibleOnScreenNotifier3D GetFirstOrDefaultNotifier()
    {
        return (VisibleOnScreenNotifier3D)GetChildren().FirstOrDefault(c => c is VisibleOnScreenNotifier3D);
    }

    private float GetSegmentLength()
    {
        return RopeLength / (SimulationParticles - 1);
    }

    private void ResetRopeRotation()
    {
        // NOTE: rope doesn't draw from origin to attach_end_to correctly if rotated
        // calling to_local() in the drawing code is too slow
        GlobalTransform = new Transform3D(Basis.Identity, GlobalPosition);
    }

    #endregion

    #region Draw

    private static void CatmullInterpolate(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float tension, float t, out Vector3 point, out Vector3 tangent)
    {
        // this is fast catmull spline

        var tSqr = t * t;
        var tCube = tSqr * t;

        var m1 = (1f - tension) / 2f * (p2 - p0);
        var m2 = (1f - tension) / 2f * (p3 - p1);

        var a = 2f * (p1 - p2) + m1 + m2;
        var b = -3f * (p1 - p2) - 2f * m1 - m2;

        point = a * tCube + b * tSqr + m1 * t + p1;
        tangent = (3f * a * tSqr + 2f * b * t + m1).Normalized();
    }

    private void DrawQuad(IReadOnlyList<Vector3> vertices, Vector3 normal, float uvx0, float uvx1)
    {
        // NOTE: still may need tangents setup for normal mapping, not tested
        // SetTangent(new Plane(-t, 0.0f));
        _mesh.SurfaceSetNormal(normal);
        _mesh.SurfaceSetUV(new Vector2(uvx0, 0.0f));
        _mesh.SurfaceAddVertex(vertices[0]);
        _mesh.SurfaceSetUV(new Vector2(uvx1, 0.0f));
        _mesh.SurfaceAddVertex(vertices[1]);
        _mesh.SurfaceSetUV(new Vector2(uvx1, 1.0f));
        _mesh.SurfaceAddVertex(vertices[2]);
        _mesh.SurfaceSetUV(new Vector2(uvx0, 0.0f));
        _mesh.SurfaceAddVertex(vertices[0]);
        _mesh.SurfaceSetUV(new Vector2(uvx1, 1.0f));
        _mesh.SurfaceAddVertex(vertices[2]);
        _mesh.SurfaceSetUV(new Vector2(uvx0, 1.0f));
        _mesh.SurfaceAddVertex(vertices[3]);
    }

    private float GetDrawSubdivisionStep(Vector3 cameraPosition, int particleIndex)
    {
        var camDistParticle = cameraPosition - _particleData[particleIndex].PositionCurrent;

        if (camDistParticle.LengthSquared() > SubdivisionLodDistance * SubdivisionLodDistance)
        {
            return 1.0f;
        }

        var tangentDots = _particleData[particleIndex].Tangent.Dot(_particleData[particleIndex + 1].Tangent);

        return
            tangentDots >= Cos5Deg ? 1.0f :
            tangentDots >= Cos15Deg ? 0.5f :
            tangentDots >= Cos30Deg ? 0.33333f :
            0.25f;
    }

    private void CalculateRopeCameraOrientation()
    {
        var cameraPosition = _camera?.GlobalPosition ?? Vector3.Zero;

        ref var start = ref _particleData[0];
        start.Tangent = (_particleData[1].PositionCurrent - start.PositionCurrent).Normalized();
        start.Normal = (start.PositionCurrent - cameraPosition).Normalized();
        start.Binormal = start.Normal.Cross(start.Tangent).Normalized();

        ref var end = ref _particleData[SimulationParticles - 1];
        end.Tangent = (end.PositionCurrent - _particleData[SimulationParticles - 2].PositionCurrent).Normalized();
        end.Normal = (end.PositionCurrent - cameraPosition).Normalized();
        end.Binormal = end.Normal.Cross(end.Tangent).Normalized();

        for (var i = 1; i < SimulationParticles - 1; i++)
        {
            ref var particle = ref _particleData[i];
            particle.Tangent = (_particleData[i + 1].PositionCurrent - _particleData[i - 1].PositionCurrent).Normalized();
            particle.Normal = (_particleData[i].PositionCurrent - cameraPosition).Normalized();
            particle.Binormal = _particleData[i].Normal.Cross(_particleData[i].Tangent).Normalized();
        }
    }

    #endregion

    #region Constraints

    private void StiffRope()
    {
        for (var iteration = 0; iteration < Iterations; iteration++)
        {
            for (var i = 0; i < SimulationParticles - 1; i++)
            {
                var segment = _particleData[i + 1].PositionCurrent - _particleData[i].PositionCurrent;
                var stretch = segment.Length() - GetSegmentLength();
                var direction = segment.Normalized();

                if (_particleData[i].IsAttached)
                {
                    _particleData[i + 1].PositionCurrent -= direction * stretch * Stiffness;
                }
                else if (_particleData[i + 1].IsAttached)
                {
                    _particleData[i].PositionCurrent += direction * stretch * Stiffness;
                }
                else
                {
                    _particleData[i].PositionCurrent += direction * stretch * 0.5f * Stiffness;
                    _particleData[i + 1].PositionCurrent -= direction * stretch * 0.5f * Stiffness;
                }
            }
        }
    }

    private bool IsRopeCollides()
    {
        var visuals = GetAabb();

        if (visuals.Size == Vector3.Zero)
        {
            return false;
        }

        _collisionCheckBox.Size = visuals.Size;
        _collisionCheckParameters.Transform = new Transform3D(_collisionCheckParameters.Transform.Basis, GlobalPosition + visuals.Position + visuals.Size / 2);

        var collisions = _spaceState.IntersectShape(_collisionCheckParameters, 1);
        return collisions.Count > 0;
    }

    private void CollideRope()
    {
        for (var i = 0; i < SimulationParticles - 1; ++i)
        {
            var result = _spaceState.IntersectRay(new PhysicsRayQueryParameters3D
            {
                From = _particleData[i].PositionCurrent + _previousNormal * CollisionCheckLength,
                To = _particleData[i + 1].PositionCurrent,
                CollisionMask = CollisionMask
            });

            if (result.Count == 0)
            {
                continue;
            }

            _previousNormal = result[NormalParameter].AsVector3();
            var yDifference = result[PositionParameter].AsVector3() - _particleData[i + 1].PositionCurrent;
            yDifference = yDifference.Project(_previousNormal);

            _particleData[i + 1].PositionCurrent += yDifference;
            _particleData[i + 1].PositionPrevious = _particleData[i + 1].PositionCurrent;
        }
    }

    #endregion

    private void DrawCurve()
    {
        // Catmull curve

        _mesh.SurfaceBegin(Mesh.PrimitiveType.Triangles);

        var cameraPosition = _camera?.GlobalPosition ?? Vector3.Zero;

        for (var i = 0; i < SimulationParticles - 1; i++)
        {
            var (p0, p1, p2, p3) = GetSimulationParticles(i);
            var step = GetDrawSubdivisionStep(cameraPosition, i);
            var t = 0.0f;

            while (t <= 1.0f)
            {
                CatmullInterpolate(p0, p1, p2, p3, 0.0f, t, out var currentPosition, out var currentTangent);
                CatmullInterpolate(p0, p1, p2, p3, 0.0f, Mathf.Min(t + step, 1.0f), out var nextPosition, out var nextTangent);

                var currentNormal = (currentPosition - cameraPosition).Normalized();
                var currentBinormal = currentNormal.Cross(currentTangent).Normalized();
                currentPosition -= GlobalPosition;

                var nextNormal = (nextPosition - cameraPosition).Normalized();
                var nextBinormal = nextNormal.Cross(nextTangent).Normalized();
                nextPosition -= GlobalPosition;

                var vs = new[]
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

    private void VerletProcess(float delta)
    {
        for (var i = 0; i < SimulationParticles; i++)
        {
            ref var p = ref _particleData[i];

            if (p.IsAttached)
            {
                continue;
            }

            var positionCurrentCopy = p.PositionCurrent;
            p.PositionCurrent = 2f * p.PositionCurrent - p.PositionPrevious + delta * delta * p.Acceleration;
            p.PositionPrevious = positionCurrentCopy;
        }
    }

    private void ApplyForces()
    {
        for (var i = 0; i < SimulationParticles; i++)
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

    private void ApplyConstraints()
    {
        StiffRope();

        if (ApplyCollision && IsRopeCollides())
        {
            CollideRope();
        }
    }

    #endregion

    public override string[] _GetConfigurationWarnings()
    {
        _visibleNotifier = GetFirstOrDefaultNotifier();
        return _visibleNotifier == null ? new[] { NoNotifierWarning } : Array.Empty<string>();
    }

    public override void _Ready()
    {
        if (!Engine.IsEditorHint() && StartDrawSimulationOnStart)
        {
            Draw = true;
            Simulate = true;
        }

        _mesh = Mesh as ImmediateMesh;
        if (_mesh == null || _mesh.GetMeta(CreationStampMeta, 0ul).AsUInt64() != GetInstanceId())
        {
            Mesh = _mesh = new ImmediateMesh();
            _mesh.SetMeta(CreationStampMeta, GetInstanceId());
            _mesh.ResourceLocalToScene = true;
        }

        _visibleNotifier = GetFirstOrDefaultNotifier();
        if (_visibleNotifier != null)
        {
            _visibleNotifier.ScreenEntered += () => Draw = true;
            _visibleNotifier.ScreenExited += () => Draw = false;
        }

        _camera = GetViewport().GetCamera3D();
        _spaceState = GetWorld3D().DirectSpaceState;
        var visuals = GetAabb();

        _collisionCheckBox = new BoxShape3D
        {
            Size = visuals.Size
        };

        _collisionCheckParameters = new PhysicsShapeQueryParameters3D
        {
            ShapeRid = _collisionCheckBox.GetRid(),
            CollisionMask = CollisionMask,
            Margin = 0.1f
        };

        _collisionCheckParameters.Transform = new Transform3D(_collisionCheckParameters.Transform.Basis, GlobalPosition + visuals.Position + visuals.Size / 2);
        MaterialOverride ??= GD.Load<StandardMaterial3D>(DefaultMaterialPath);

        CreateRope();
    }

    public override void _PhysicsProcess(double delta)
    {
        if (Engine.IsEditorHint() && _particleData == null)
        {
            CreateRope();
        }

        _time += delta;
        _simulationDelta += delta;

        var simulationStep = 1 / (float) SimulationRate;
        if (_simulationDelta < simulationStep)
        {
            return;
        }

        if (_attachEnd != null)
        {
            ref var end = ref _particleData![SimulationParticles - 1];
            end.PositionCurrent = _attachEnd.GlobalPosition;
        }

        if (AttachStart)
        {
            ref var start = ref _particleData![0];
            start.PositionCurrent = GlobalPosition;
        }

        if (Simulate)
        {
            ApplyForces();
            VerletProcess((float) _simulationDelta);
            ApplyConstraints();
        }

        if (Draw)
        {
            _camera = GetViewport().GetCamera3D();
            CalculateRopeCameraOrientation();
            _mesh.ClearSurfaces();
            ResetRopeRotation();
            DrawRopeDebugParticles();
            DrawCurve();

            if (_visibleNotifier != null)
            {
                _visibleNotifier.Aabb = GetAabb();
            }
        }

        EmitSignal(SignalName.SimulationStep, _simulationDelta);
        _simulationDelta = 0;
    }

    public void CreateRope()
    {
        var endLocation = _attachEnd?.GlobalPosition ?? GlobalPosition + Vector3.Down * RopeLength;
        var acceleration = Gravity * GravityScale;
        var segment = GetSegmentLength();
        _particleData = RopeParticleData.GenerateParticleData(endLocation, GlobalPosition, acceleration, _simulationParticles, segment);

        ref var start = ref _particleData[0];
        ref var end = ref _particleData[SimulationParticles - 1];

        start.IsAttached = AttachStart;
        end.IsAttached = _attachEnd != null;
        end.PositionPrevious = endLocation;
        end.PositionCurrent = endLocation;

        for (var i = 0; i < PreprocessIterations; i++)
        {
            VerletProcess(PreprocessDelta);
            ApplyConstraints();
        }

        CalculateRopeCameraOrientation();
    }

    public void DestroyRope()
    {
        _particleData.Resize(0);
        SimulationParticles = 0;
    }

    public void DrawRopeDebugParticles()
    {
        const float debugParticleLength = 0.3f;

        if (!DrawDebugParticles)
        {
            return;
        }

        _mesh.SurfaceBegin(Mesh.PrimitiveType.Lines);

        for (var i = 0; i < SimulationParticles; i++)
        {
            var particle = _particleData[i];
            var localPosition = particle.PositionCurrent - GlobalPosition;

            _mesh.SurfaceAddVertex(localPosition);
            _mesh.SurfaceAddVertex(localPosition + debugParticleLength * particle.Tangent);

            _mesh.SurfaceAddVertex(localPosition);
            _mesh.SurfaceAddVertex(localPosition + debugParticleLength * particle.Normal);

            _mesh.SurfaceAddVertex(localPosition);
            _mesh.SurfaceAddVertex(localPosition + debugParticleLength * particle.Binormal);
        }

        _mesh.SurfaceEnd();
    }
}
