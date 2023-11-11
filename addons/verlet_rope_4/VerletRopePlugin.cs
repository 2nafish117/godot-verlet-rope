#if TOOLS

using Godot;

namespace VerletRope4;

[Tool]
public partial class VerletRopePlugin : EditorPlugin
{
    private const string CustomTypeName = "VerletRope";
    private const string ScriptPath = "res://addons/verlet_rope_4/VerletRope.cs";
    private const string IconPath = "res://addons/verlet_rope_4/icon.svg";

    public override void _EnterTree()
    {
        var script = GD.Load<Script>(ScriptPath);
        var texture = GD.Load<Texture2D>(IconPath);
        AddCustomType(CustomTypeName, nameof(MeshInstance3D), script, texture);
    }

    public override void _ExitTree()
    {
        RemoveCustomType(CustomTypeName);
    }
}

#endif
