tool
extends EditorPlugin

func _enter_tree():
	add_custom_type("VerletRope", "ImmediateGeometry", preload("res://addons/verlet_rope_gd/verlet_rope.gd"), preload("res://icon.png"))
	print_debug("verlet_rope_gd loaded sucessfully")

func _exit_tree():
	remove_custom_type("VerletRope")
	print_debug("verlet_rope_gd un-loaded sucessfully")
