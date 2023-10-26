using Godot;

namespace VerletRope4;

public partial class MovingRopeDemo : Node3D
{
    private static readonly NodePath PositionProperty = new(Node3D.PropertyName.Position);

    private bool _moveForward;

    [Export] public VerletRope RopeMoving { get; set; }
    [Export] public Vector3 MovingPath { get; set; }
    [Export] public float PathTime{ get; set; }
    [Export] public Tween.TransitionType TransitionType { get; set; }

    private void MoveRope()
    {
        _moveForward = !_moveForward;
        var sign = _moveForward ? 1 : -1;

        var tween = RopeMoving.CreateTween().SetTrans(TransitionType);
        tween.TweenProperty(RopeMoving, PositionProperty, sign * MovingPath, PathTime).AsRelative();
        tween.TweenCallback(Callable.From(MoveRope));
    }

    public override void _Ready()
    {
        MoveRope();
    }
}
