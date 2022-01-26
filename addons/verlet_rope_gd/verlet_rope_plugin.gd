tool
extends EditorPlugin

func _enter_tree():
	add_custom_type("VerletRopeGd", "ImmediateGeometry", preload("./verlet_rope.gd"), preload("./icon_verlet_rope_gd.svg"))
	print_debug("verlet_rope_gd loaded sucessfully")

func _exit_tree():
	remove_custom_type("VerletRopeGd")
	print_debug("verlet_rope_gd un-loaded sucessfully")
