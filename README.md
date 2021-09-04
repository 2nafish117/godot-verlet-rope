# Godot Verlet Rope

A fast implementation of verlet integration based rope pysics, similar to the one seen in half life 2.

# Quick Start
1. Add either rope node to the scene and set the AttachEndTo property in the inspector to any Spatial derived node (hint: use another rope node to chain the ropes or a position node to end the chain).
2. To use a custom rope texture you will have to set the Albedo of the material in the inpector. Use a tiling texture and set the UV0 x property to change the tiling.
3. The rope material has to set Cull Mode to disabled for the rope to render on both sides.
4. Play around with the rope parameters.
5. If you do make changes to GDVerletRope.gd or CSVerletRope.cs make sure to close and reopen the scene in the editor to stop some errors from piling up in the logs.

# Featues
1. Verlet integration based particle simulation
2. ImmediateGeometry to draw the rope
3. Set number of particles, length, width and iterations for the rope.
4. PreprocessIterations processes the rope in _ready so it doesnt start out in a non rest position, a value of 20 is good enough, a higher value makes the scene load time longer.
5. Always faces the current camera in play mode
6. Automatically tesselates some sharp parts of the rope using Catmull rom splines to keep the rope looking smooth (use the SubdivLodDistance to change beyond what distance this tesselation stops).
7. Add a visibility notifier as a child to enable and disable drawing of the rope when the current camera is not viewing the rope. The rope automatically grows and shrinks the aabb, so you dont have to set the extents of the VisibilityNotifier yourself).
8. Enable/disable/configure forces like gravity, wind, air drag.
9. Rudimentary collisions using raycasts (only tests collisions if a collider enters its aabb), you need to enable this with ApplyCollisions.

