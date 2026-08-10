## perception.gd  (class DronePerception)
## Attach as a CHILD of DroneBody at the desired camera mount pose (this node's
## global_transform IS the camera pose; -Z is forward, matching Godot cameras).
##
## Two sensors:
##  - Depth + semantics via RAYCASTS — works in --headless, deterministic, cheap.
##    This is the primary sensor for obstacle/nav learning.
##  - RGB via a SubViewport — needs a real GPU/display context (NOT --headless;
##    use a windowed run or Xvfb on a server). Optional, off by default.

extends Node3D
class_name DronePerception

@export_group("Depth (raycast)")
@export var depth_cols: int = 32
@export var depth_rows: int = 24
@export_range(10.0, 170.0) var h_fov_deg: float = 90.0
@export_range(10.0, 170.0) var v_fov_deg: float = 70.0
@export var max_range: float = 40.0

@export_group("RGB (needs GPU context)")
@export var enable_rgb: bool = false
@export var rgb_width: int = 128
@export var rgb_height: int = 96

var _vp: SubViewport = null
var _cam: Camera3D = null

func _ready() -> void:
	if enable_rgb:
		_setup_rgb()

func _setup_rgb() -> void:
	_vp = SubViewport.new()
	_vp.size = Vector2i(rgb_width, rgb_height)
	_vp.world_3d = get_viewport().world_3d          # render the MAIN scene
	_vp.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	add_child(_vp)
	_cam = Camera3D.new()
	_cam.fov = v_fov_deg
	_vp.add_child(_cam)
	_cam.current = true

func _process(_delta: float) -> void:
	if _cam:
		_cam.global_transform = global_transform    # keep RGB cam on the mount

# --------------------------------------------------------------------------
# Depth + semantics (raycast grid). Row 0 = top of image.
# Returns ranges in metres (max_range = no hit) and a small class id per ray.
# --------------------------------------------------------------------------
func sample_depth() -> Dictionary:
	var space := get_world_3d().direct_space_state
	var t := global_transform
	var origin := t.origin
	var th := tan(deg_to_rad(h_fov_deg) * 0.5)
	var tv := tan(deg_to_rad(v_fov_deg) * 0.5)

	var exclude: Array[RID] = []
	var parent := get_parent()
	if parent and parent.has_method("get_rid"):
		exclude.append(parent.get_rid())

	var ranges: Array = []
	var classes: Array = []
	ranges.resize(depth_cols * depth_rows)
	classes.resize(depth_cols * depth_rows)

	var idx := 0
	for j in depth_rows:
		var v := ((float(j) + 0.5) / float(depth_rows)) * 2.0 - 1.0
		for i in depth_cols:
			var u := ((float(i) + 0.5) / float(depth_cols)) * 2.0 - 1.0
			var dir_cam := Vector3(u * th, -v * tv, -1.0).normalized()
			var dir := t.basis * dir_cam
			var q := PhysicsRayQueryParameters3D.create(origin, origin + dir * max_range)
			q.exclude = exclude
			var hit := space.intersect_ray(q)
			if hit.is_empty():
				ranges[idx] = max_range
				classes[idx] = 0
			else:
				ranges[idx] = origin.distance_to(hit.position)
				classes[idx] = _class_of(hit.collider)
			idx += 1

	return {
		"cols": depth_cols, "rows": depth_rows, "max_range": max_range,
		"ranges": ranges, "classes": classes,
	}

func _class_of(col) -> int:
	if col == null:
		return 0
	if col.is_in_group("obstacle"):
		return 1
	if col.is_in_group("ground"):
		return 2
	if col.is_in_group("goal"):
		return 3
	return 4

# --------------------------------------------------------------------------
# RGB. Returns {} in --headless (no render). Encoding is raw RGB8, base64.
# --------------------------------------------------------------------------
func sample_rgb() -> Dictionary:
	if _vp == null:
		return {}
	var tex := _vp.get_texture()
	if tex == null:
		return {}
	var img := tex.get_image()
	if img == null or img.is_empty():
		return {}
	img.convert(Image.FORMAT_RGB8)
	return {
		"w": img.get_width(), "h": img.get_height(),
		"encoding": "raw_rgb8",
		"data": Marshalls.raw_to_base64(img.get_data()),
	}
