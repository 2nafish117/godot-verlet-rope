# Godot 4 Verlet Rope (.NET)

It is a port to Godot 4.0+ of [the fast implementation of verlet-integration-based rope physics](https://github.com/2nafish117/godot-verlet-rope), similar to the one seen in Half-Life 2. 

The port didn't change much on the functionality side: added the possibility to enable ropes at the start of the game (to avoid having ropes enabled in the editor all the time) and unfortunately had to remove the simulation rate as it was causing issues in Godot 4 (I probably will try to bring it back later). All other changes involve rebasing to the Godot 4 API and heavy refactoring. The code should now be much more readable and adhere to C# guidelines, so feel free to read/modify it.

That's it, enjoy cool ropes! <sub>(C) 2023 Timofey Ivanov / tshmofen</sub>

![](https://github.com/2nafish117/godot-verlet-rope/blob/master/images/Screenshot.png)

# Hints
1. Try attaching to another rope node to chain the ropes.
2. To use textures enable tiling textures and set the UV0 property to change the tiling.
3. Remember to set rope's material cull mode to disabled, otherwise rope will be rendered only on one side (It is mostly for editor clarity, as during runtime ropes will be always facing the camera).
4. Using one saved resource-noise allows for a global rope wind.

# Notes
1. Rotations on ropes are disabled as they are not working with the algorithm correctly. Anyways, they are not really needed, that is enough to just move the endpoints instead.
2. If you see any errors piling up in the logs after any changes in the script, just close/reopen the scene in the editor.

# Featues
1. [Verlet Integration](https://en.wikipedia.org/wiki/Verlet_integration) based particle simulation.
2. Full rope simulation within the editor, set Simulate to off to stop the simulation. 
3. Set number of particles, length, width and iterations for the rope.
4. Always faces the current camera in play mode.
5. Automatically tesselates some sharp parts of the rope using Catmull rom splines to keep the rope looking smooth (`SubdivisionLodDistance`).
6. Switches drawing of the rope when it is not visible on the current camera (When `VisibleOnScreenNofitier3D` is attached to the rope).
7. Intregration of forces as gravity, wind and air damping.
8. Basic collisions using raycasts.

# Rope export params and functions

| Export variable | How it works |
|--|--|
| Attach Start   | Determines if the start point is fixed in place. |
| Attach End     | A link to any Node3D, if it is set up, the end of the rope will be folliwing this node. |
| Rope Length    | Length. |
| Rope Width     | Width. Ropes are flat, but always look at the camera, so width effectively behaves as a diameter.|
| Simulation Particles | Number of particles to simulate the rope. Odd number (greater than 3) is recommended for ropes attached on both sides for a smoother rope at its lowest point. |
| Iterations     | Number of verlet constraint iterations per frame, higher value gives accurate rope simulation for lengthy ropes with many simulation particles. Increase if you find the rope is sagging or stretching too much. |
| Preprocess Iterations| Number of iterations to be precalculated in `_ready()` to set the rope in a rest position. Value of 20-30 should be enough. |
| Stiffness      | AKA elasticity - it is a fraction that controls how much the verlet constraint corrects the rope. value from 0.1 to 1.0 is recommended. |
| Simulate       | Enables the simulation. Rope is still being drawn every frame if this is off. |
| Draw           | Enables the mesh drawing, you will still see the rope because `ImmediateGeometry.clear` wasnt called, but the rope isnt being drawn every frame. Rope is still being simulated if this is off. |
| Start Draw Simulation On Start | Will enable Simulate and Draw on the start of the game. Useful to not have moving ropes in editor. |
| Subdivision Lod Distance | Sets max distance where Catmull-Rom spline smoothing is applied for required segments. |
| Apply Gravity  | Enables gravity. |
| Gravity        | Gravity direction vector. |
| Gravity Scale  | A factor to uniformly scale the gravity vector. |
| Apply Wind     | Applies wind noise. |
| Wind Noise     | Noise as a base for wind, noise period controls the turbulence (kinda). Use saved resource across different ropes for a global wind setting. |
| Wind           | The wind direction vector.|
| Wind Scale     | A factor to scale the wind direction. |
| Apply Damping  | Enables drag/damping. May help when rope bugs out by bringing it back to rest. |
| Damping Factor | Amount of damping. |
| Apply Collision| Enables collision with bodies. Collisions work best on smooth surfaces without sharp edges. |
| Collision Mask | The collision mask to be used for collisions. |
