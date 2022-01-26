#if TOOLS
using Godot;

namespace Fish {
	[Tool]
	public class VerletRopePlugin : EditorPlugin
	{
		public override void _EnterTree()
		{
			// C# cant load from relative file paths :(
			Script script = GD.Load<Script>("res://addons/verlet_rope_cs/VerletRopeCs.cs");
			Texture texture = GD.Load<Texture>("res://addons/verlet_rope_cs/icon_verlet_rope_cs.svg");
			AddCustomType("VerletRopeCs", "ImmediateGeometry", script, texture);
			GD.Print("plugin Verlet_rope_cs loaded");
		}

		public override void _ExitTree()
		{
			RemoveCustomType("VerletRopeCs");
			GD.Print("plugin Verlet_rope_cs un-loaded");
		}
	}
}
#endif