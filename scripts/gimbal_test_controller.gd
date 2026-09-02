## gimbal_test_controller.gd
## Validates single GIMBAL_MASTER module with Yaw, Pitch, Roll articulation

extends Node3D

@onready var gimbal_yaw = $GIMBAL_MASTER.find_child("GIMBAL_YAW", true, false)
@onready var gimbal_pitch = $GIMBAL_MASTER.find_child("GIMBAL_PITCH", true, false)
@onready var gimbal_roll = $GIMBAL_MASTER.find_child("GIMBAL_ROLL", true, false)

@onready var camera_pivot = $CameraPivot
@onready var camera = $CameraPivot/Camera3D

var is_dragging: bool = false
var last_mouse_pos: Vector2 = Vector2.ZERO
var cam_yaw: float = 25.0
var cam_pitch: float = -15.0
var cam_dist: float = 0.65

func _ready() -> void:
	_update_cam()

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		if event.button_index in [MOUSE_BUTTON_LEFT, MOUSE_BUTTON_RIGHT]:
			is_dragging = event.pressed
			last_mouse_pos = event.position
		elif event.button_index == MOUSE_BUTTON_WHEEL_UP:
			cam_dist = clampf(cam_dist - 0.04, 0.15, 2.0)
			_update_cam()
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			cam_dist = clampf(cam_dist + 0.04, 0.15, 2.0)
			_update_cam()
	elif event is InputEventMouseMotion and is_dragging:
		var delta = event.position - last_mouse_pos
		last_mouse_pos = event.position
		cam_yaw -= delta.x * 0.5
		cam_pitch = clampf(cam_pitch - delta.y * 0.4, -85.0, 85.0)
		_update_cam()

func _update_cam() -> void:
	if camera_pivot:
		camera_pivot.rotation_degrees = Vector3(cam_pitch, cam_yaw, 0)
	if camera:
		camera.position = Vector3(0, 0, cam_dist)

func _on_slider_yaw_value_changed(value: float) -> void:
	if is_instance_valid(gimbal_yaw):
		gimbal_yaw.rotation_degrees.y = value

func _on_slider_pitch_value_changed(value: float) -> void:
	if is_instance_valid(gimbal_pitch):
		gimbal_pitch.rotation_degrees.x = value

func _on_slider_roll_value_changed(value: float) -> void:
	if is_instance_valid(gimbal_roll):
		gimbal_roll.rotation_degrees.z = value
