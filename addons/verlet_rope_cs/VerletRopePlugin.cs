#if TOOLS
using Godot;
using System;

namespace Fish {
	[Tool]
	public class VerletRopePlugin : EditorPlugin
	{
		public override void _EnterTree()
		{
			Script script = GD.Load<Script>("res://addons/verlet_rope/VerletRope.cs");
			Texture texture = GD.Load<Texture>("res://icon.png");
			AddCustomType("VerletRope", "ImmediateGeometry", script, texture);
			GD.Print("plugin Verlet_rope_cs loaded");
		}

		public override void _ExitTree()
		{
			RemoveCustomType("VerletRope");
			GD.Print("plugin Verlet_rope_cs un-loaded");
		}
	}
}
#endif