## rotor_test_controller.gd
## Validates single ROTOR_MASTER module at 0, 500, 1000, 2000, 3000 RPM in CW and CCW directions

extends Node3D

@onready var rotor_model = $ROTOR_MASTER
@onready var camera_pivot = $CameraPivot
@onready var camera = $CameraPivot/Camera3D
@onready var lbl_rpm = $UI/Panel/LblRPM
@onready var lbl_dir = $UI/Panel/LblDir

var target_rpm: float = 0.0
var rotation_direction: float = 1.0 # 1.0 = CCW, -1.0 = CW around local +Y

var is_dragging: bool = false
var last_mouse_pos: Vector2 = Vector2.ZERO
var cam_yaw: float = 30.0
var cam_pitch: float = -20.0
var cam_dist: float = 0.85

func _ready() -> void:
	_update_cam()
	_update_labels()

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		if event.button_index in [MOUSE_BUTTON_LEFT, MOUSE_BUTTON_RIGHT]:
			is_dragging = event.pressed
			last_mouse_pos = event.position
		elif event.button_index == MOUSE_BUTTON_WHEEL_UP:
			cam_dist = clampf(cam_dist - 0.05, 0.20, 2.5)
			_update_cam()
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			cam_dist = clampf(cam_dist + 0.05, 0.20, 2.5)
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

func _process(delta: float) -> void:
	if is_instance_valid(rotor_model) and target_rpm > 0.0:
		var rad_s = (target_rpm / 60.0) * TAU * rotation_direction
		rotor_model.rotate_y(rad_s * delta)

func _update_labels() -> void:
	if lbl_rpm:
		lbl_rpm.text = "RPM: %d" % int(target_rpm)
	if lbl_dir:
		lbl_dir.text = "DIRECTION: %s" % ("CCW (Counter-Clockwise)" if rotation_direction > 0 else "CW (Clockwise)")

func set_rpm(rpm: float) -> void:
	target_rpm = rpm
	_update_labels()

func toggle_direction() -> void:
	rotation_direction *= -1.0
	_update_labels()
