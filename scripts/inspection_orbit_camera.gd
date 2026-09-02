## inspection_orbit_camera.gd
## Shared professional inspection camera with Orbit, Pan, Zoom, and 8 Preset Angles.

extends Node3D

@export var target_component_name: String = "COMPONENT"
@export var default_distance: float = 0.85
@export var min_distance: float = 0.15
@export var max_distance: float = 3.5

@onready var pivot: Node3D = self
@onready var camera: Camera3D = $Camera3D

var cam_yaw: float = 35.0
var cam_pitch: float = -20.0
var cam_distance: float = 0.85
var target_offset: Vector3 = Vector3.ZERO

var is_orbiting: bool = false
var is_panning: bool = false
var last_mouse_pos: Vector2 = Vector2.ZERO

func _ready() -> void:
	cam_distance = default_distance
	_update_camera_transform()

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_LEFT:
			is_orbiting = event.pressed
			last_mouse_pos = event.position
		elif event.button_index == MOUSE_BUTTON_RIGHT or event.button_index == MOUSE_BUTTON_MIDDLE:
			is_panning = event.pressed
			last_mouse_pos = event.position
		elif event.button_index == MOUSE_BUTTON_WHEEL_UP:
			cam_distance = clampf(cam_distance - 0.05 * cam_distance, min_distance, max_distance)
			_update_camera_transform()
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			cam_distance = clampf(cam_distance + 0.05 * cam_distance, min_distance, max_distance)
			_update_camera_transform()
	elif event is InputEventMouseMotion:
		var delta = event.position - last_mouse_pos
		last_mouse_pos = event.position
		if is_orbiting:
			cam_yaw -= delta.x * 0.4
			cam_pitch = clampf(cam_pitch - delta.y * 0.3, -88.0, 88.0)
			_update_camera_transform()
		elif is_panning:
			var pan_speed = cam_distance * 0.002
			var right_dir = camera.global_transform.basis.x
			var up_dir = camera.global_transform.basis.y
			target_offset -= right_dir * delta.x * pan_speed - up_dir * delta.y * pan_speed
			_update_camera_transform()
	elif event is InputEventKey and event.pressed:
		if event.keycode == KEY_F:
			reset_focus()

func _update_camera_transform() -> void:
	pivot.position = target_offset
	pivot.rotation_degrees = Vector3(cam_pitch, cam_yaw, 0.0)
	if camera:
		camera.position = Vector3(0, 0, cam_distance)

func reset_focus() -> void:
	target_offset = Vector3.ZERO
	cam_distance = default_distance
	cam_yaw = 35.0
	cam_pitch = -20.0
	_update_camera_transform()

func set_preset(preset_name: String) -> void:
	target_offset = Vector3.ZERO
	match preset_name:
		"FRONT":
			cam_yaw = 0.0
			cam_pitch = 0.0
		"REAR":
			cam_yaw = 180.0
			cam_pitch = 0.0
		"LEFT":
			cam_yaw = -90.0
			cam_pitch = 0.0
		"RIGHT":
			cam_yaw = 90.0
			cam_pitch = 0.0
		"TOP":
			cam_yaw = 0.0
			cam_pitch = -88.0
		"BOTTOM":
			cam_yaw = 0.0
			cam_pitch = 88.0
		"45_FRONT":
			cam_yaw = 45.0
			cam_pitch = -25.0
		"45_REAR":
			cam_yaw = 135.0
			cam_pitch = -25.0
	_update_camera_transform()
