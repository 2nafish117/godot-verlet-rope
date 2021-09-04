extends Control

class Stats:
	var _name: String
	var _object: Object
	var _property: String
	var _is_method: bool
	var _is_separator: bool

	func _init(name: String, object: Object, property: String, is_method: bool, is_separator: bool) -> void:
		_name = name
		_object = object
		_property = property
		_is_method = is_method
		_is_separator = is_separator
		
	func get_stat() -> String:
		if _is_separator:
			return "-------" + _name + "-------\n"
		else:
			var value = null
			if _object and weakref(_object).get_ref():
				if _is_method:
					value = _object.call(_property)
				else:
					value = _object.get(_property)
			return _name + ": " + str(value) + "\n"

var stats := []

var cleanup_initialised := false

func _cleanup():
	stats = []
	cleanup_initialised = false

func _startup():
	if not cleanup_initialised:
		cleanup_initialised = true
		var _err := get_tree().current_scene.connect("tree_exiting", self, "_cleanup")

# DbgStats.add_stat("velocity", self, "velocity")
func add_stat(stat_name: String, object: Object, property: String, is_method: bool = false) -> void:
	_startup()
	var stat := Stats.new(stat_name, object, property, is_method, false)
	stats.append(stat)

func add_separator(separator_title: String) -> void:
	_startup()
	var stat := Stats.new(separator_title, null, "", false, true)
	stats.append(stat)

func get_static_memory() -> String:
	return String.humanize_size(OS.get_static_memory_usage())
	
func get_dynamic_memory() -> String:
	return String.humanize_size(OS.get_dynamic_memory_usage())

func _ready() -> void:
	add_stat("fps", Engine, "get_frames_per_second", true)
	add_stat("static mem", self, "get_static_memory", true)
	add_stat("dynamic mem", self, "get_dynamic_memory", true)

func _process(_delta: float) -> void:
	if not visible:
		return
		
	$Label.text = ""
	for s in stats:
		if s._object == null and not s._is_separator:
			stats.erase(s)
		else:
			$Label.text += s.get_stat()
