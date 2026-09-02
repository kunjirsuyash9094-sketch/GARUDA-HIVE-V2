## kargil_simulation_manager.gd
## Full Interactive Cockpit Dashboard & Flight Controller with 360° Free Mouse Orbit & Macro Zoom
## Features Garuda OS Power Selector (0 RPM, ECO, BALANCED, MAX, 10,000 RPM ULTRA) & 4000m+ High Altitude Modes

extends Node3D
class_name KargilSimulationManager

var drone: Node = null
var chase_camera: Camera3D = null

# HUD Elements
var status_badge: Label = null
var battery_label: Label = null
var gps_label: Label = null
var alt_value: Label = null
var alt_target_val: Label = null
var spd_value: Label = null
var vs_value: Label = null
var rpy_value: Label = null
var power_mode_label: Label = null

# Motor Progress Bars (M1 - M8)
var motor_bars: Array[ProgressBar] = []
var motor_labels: Array[Label] = []

# Camera System
var camera_mode: int = 0 # 0=360 Free Orbit & Macro Zoom, 1=Chase Follow, 2=FPV Cockpit, 3=Tactical Top-Down
var camera_names: Array[String] = ["360° MOUSE ORBIT & ZOOM", "CHASE FOLLOW CAM", "FPV NOSE CAM", "TACTICAL TOP-DOWN"]

# Mouse Orbit & Inspection Parameters
var is_mouse_dragging: bool = false
var orbit_yaw: float = 0.0
var orbit_pitch: float = deg_to_rad(18.0)
var orbit_distance: float = 2.8
var target_orbit_yaw: float = 0.0
var target_orbit_pitch: float = deg_to_rad(18.0)
var target_orbit_distance: float = 2.8
const MOUSE_SENSITIVITY: float = 0.0055
const ZOOM_STEP: float = 0.35
const MIN_DISTANCE: float = 0.55  # Macro close-up to inspect textures
const MAX_DISTANCE: float = 45.0  # Wide field view for 4000m inspection

func _ready() -> void:
	_find_core_nodes()
	_init_hud_elements()
	_connect_interactive_buttons()
	print("[GARUDA] 🚁 Simulator Manager Ready. Bound to drone: ", drone)

func _find_core_nodes() -> void:
	drone = get_node_or_null("GARUDA_HL01")
	if drone == null:
		drone = find_child("GARUDA_HL01", true, false)
	if drone == null:
		for c in get_children():
			if c.name.contains("GARUDA") or c.has_method("get_telemetry"):
				drone = c
				break

	chase_camera = get_node_or_null("ChaseCamera")
	if chase_camera == null:
		chase_camera = find_child("ChaseCamera", true, false)

func _init_hud_elements() -> void:
	status_badge = find_child("StatusBadge", true, false)
	battery_label = find_child("BatLabel", true, false)
	gps_label = find_child("GpsLabel", true, false)
	alt_value = find_child("AltVal", true, false)
	alt_target_val = find_child("AltTgt", true, false)
	spd_value = find_child("SpdVal", true, false)
	vs_value = find_child("VsVal", true, false)
	rpy_value = find_child("RpyVal", true, false)
	power_mode_label = find_child("LblPowerMode", true, false)

	motor_bars.clear()
	motor_labels.clear()
	for i in range(8):
		var bar = find_child("M%d_Bar" % (i + 1), true, false)
		var lbl = find_child("M%d_Lbl" % (i + 1), true, false)
		if bar is ProgressBar: motor_bars.append(bar)
		if lbl is Label: motor_labels.append(lbl)

	var btn_cam = find_child("BtnCam", true, false)
	if btn_cam: btn_cam.text = "🎥 " + camera_names[camera_mode]

func _connect_interactive_buttons() -> void:
	var buttons = [
		find_child("BtnArm", true, false),
		find_child("BtnHover", true, false),
		find_child("BtnLand", true, false),
		find_child("BtnCam", true, false),
		find_child("BtnReset", true, false),
		find_child("BtnDisarm", true, false),
		find_child("BtnClimb", true, false),
		find_child("BtnDescend", true, false),
		find_child("BtnPwr0", true, false),
		find_child("BtnPwrEco", true, false),
		find_child("BtnPwrBal", true, false),
		find_child("BtnPwrMax", true, false),
		find_child("BtnPwrUltra", true, false),
		find_child("BtnClimb1000", true, false),
		find_child("BtnClimb2500", true, false),
		find_child("BtnClimb4000", true, false)
	]
	for b in buttons:
		if b is Button:
			b.focus_mode = Control.FOCUS_NONE

	var btn_arm = find_child("BtnArm", true, false)
	if btn_arm: btn_arm.pressed.connect(func(): if drone and drone.has_method("trigger_arm_takeoff"): drone.trigger_arm_takeoff())

	var btn_hover = find_child("BtnHover", true, false)
	if btn_hover: btn_hover.pressed.connect(func(): if drone and drone.has_method("trigger_auto_hover"): drone.trigger_auto_hover())

	var btn_land = find_child("BtnLand", true, false)
	if btn_land: btn_land.pressed.connect(func(): if drone and drone.has_method("trigger_rtl_landing"): drone.trigger_rtl_landing())

	var btn_cam = find_child("BtnCam", true, false)
	if btn_cam: btn_cam.pressed.connect(func(): _cycle_camera())

	var btn_reset = find_child("BtnReset", true, false)
	if btn_reset: btn_reset.pressed.connect(func(): if drone and drone.has_method("reset_to_pad"): drone.reset_to_pad())

	var btn_disarm = find_child("BtnDisarm", true, false)
	if btn_disarm: btn_disarm.pressed.connect(func(): if drone and drone.has_method("trigger_disarm"): drone.trigger_disarm())

	var btn_climb = find_child("BtnClimb", true, false)
	if btn_climb: btn_climb.pressed.connect(func():
		if drone:
			drone.armed = true
			drone.target_altitude = clamp(drone.target_altitude + 5.0, 0.5, 4500.0)
	)

	var btn_descend = find_child("BtnDescend", true, false)
	if btn_descend: btn_descend.pressed.connect(func():
		if drone:
			drone.target_altitude = max(0.5, drone.target_altitude - 5.0)
	)

	# Power Mode Connections
	var p0 = find_child("BtnPwr0", true, false)
	if p0: p0.pressed.connect(func(): if drone and drone.has_method("set_power_profile"): drone.set_power_profile("STOPPED"))

	var pe = find_child("BtnPwrEco", true, false)
	if pe: pe.pressed.connect(func(): if drone and drone.has_method("set_power_profile"): drone.set_power_profile("ECO"))

	var pb = find_child("BtnPwrBal", true, false)
	if pb: pb.pressed.connect(func(): if drone and drone.has_method("set_power_profile"): drone.set_power_profile("BALANCED"))

	var pm = find_child("BtnPwrMax", true, false)
	if pm: pm.pressed.connect(func(): if drone and drone.has_method("set_power_profile"): drone.set_power_profile("MAX"))

	var pu = find_child("BtnPwrUltra", true, false)
	if pu: pu.pressed.connect(func(): if drone and drone.has_method("set_power_profile"): drone.set_power_profile("ULTRA"))

	# Direct High-Altitude Ascent Setpoints
	var c1 = find_child("BtnClimb1000", true, false)
	if c1: c1.pressed.connect(func(): if drone and drone.has_method("set_climb_target"): drone.set_climb_target(1000.0))

	var c2 = find_child("BtnClimb2500", true, false)
	if c2: c2.pressed.connect(func(): if drone and drone.has_method("set_climb_target"): drone.set_climb_target(2500.0))

	var c4 = find_child("BtnClimb4000", true, false)
	if c4: c4.pressed.connect(func(): if drone and drone.has_method("set_climb_target"): drone.set_climb_target(4000.0))

func _cycle_camera() -> void:
	camera_mode = (camera_mode + 1) % 4
	var btn_cam = find_child("BtnCam", true, false)
	if btn_cam: btn_cam.text = "🎥 " + camera_names[camera_mode]

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_LEFT or event.button_index == MOUSE_BUTTON_RIGHT or event.button_index == MOUSE_BUTTON_MIDDLE:
			is_mouse_dragging = event.pressed
		elif event.button_index == MOUSE_BUTTON_WHEEL_UP and event.pressed:
			target_orbit_distance = clamp(target_orbit_distance - ZOOM_STEP * (target_orbit_distance / 2.0), MIN_DISTANCE, MAX_DISTANCE)
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN and event.pressed:
			target_orbit_distance = clamp(target_orbit_distance + ZOOM_STEP * (target_orbit_distance / 2.0), MIN_DISTANCE, MAX_DISTANCE)

	elif event is InputEventMouseMotion and is_mouse_dragging:
		target_orbit_yaw -= event.relative.x * MOUSE_SENSITIVITY
		target_orbit_pitch = clamp(target_orbit_pitch + event.relative.y * MOUSE_SENSITIVITY, deg_to_rad(-85.0), deg_to_rad(85.0))

	elif event is InputEventKey and event.pressed:
		if event.keycode == KEY_SPACE:
			if drone and drone.has_method("trigger_arm_takeoff"): drone.trigger_arm_takeoff()
		elif event.keycode == KEY_H:
			if drone and drone.has_method("trigger_auto_hover"): drone.trigger_auto_hover()
		elif event.keycode == KEY_L:
			if drone and drone.has_method("trigger_rtl_landing"): drone.trigger_rtl_landing()
		elif event.keycode == KEY_C:
			_cycle_camera()
		elif event.keycode == KEY_R:
			if drone and drone.has_method("reset_to_pad"): drone.reset_to_pad()
		elif event.keycode == KEY_BACKSPACE or event.keycode == KEY_ESCAPE:
			if drone and drone.has_method("trigger_disarm"): drone.trigger_disarm()
		elif event.keycode == KEY_1:
			if drone and drone.has_method("set_power_profile"): drone.set_power_profile("STOPPED")
		elif event.keycode == KEY_2:
			if drone and drone.has_method("set_power_profile"): drone.set_power_profile("ECO")
		elif event.keycode == KEY_3:
			if drone and drone.has_method("set_power_profile"): drone.set_power_profile("BALANCED")
		elif event.keycode == KEY_4:
			if drone and drone.has_method("set_power_profile"): drone.set_power_profile("MAX")
		elif event.keycode == KEY_5:
			if drone and drone.has_method("set_power_profile"): drone.set_power_profile("ULTRA")

func _process(delta: float) -> void:
	_handle_flight_keyboard_input()
	_update_camera(delta)
	_update_dashboard_telemetry()

func _handle_flight_keyboard_input() -> void:
	if not drone: return

	var w = Input.is_key_pressed(KEY_W)
	var s = Input.is_key_pressed(KEY_S)
	var up = Input.is_key_pressed(KEY_UP)
	var down = Input.is_key_pressed(KEY_DOWN)
	var left = Input.is_key_pressed(KEY_LEFT)
	var right = Input.is_key_pressed(KEY_RIGHT)
	var a = Input.is_key_pressed(KEY_A)
	var d = Input.is_key_pressed(KEY_D)

	# Altitude Climb / Descend
	if w:
		drone.input_climb = 1.0
		drone.is_landing = false
	elif s:
		drone.input_climb = -1.0
		drone.is_landing = false
	else:
		drone.input_climb = 0.0

	# Pitch (Forward / Backward)
	if up:
		drone.input_forward = -1.0
		drone.is_landing = false
	elif down:
		drone.input_forward = 1.0
		drone.is_landing = false
	else:
		drone.input_forward = 0.0

	# Roll (Strafe Left / Right)
	if left:
		drone.input_strafe = -1.0
		drone.is_landing = false
	elif right:
		drone.input_strafe = 1.0
		drone.is_landing = false
	else:
		drone.input_strafe = 0.0

	# Yaw (Turn Left / Right)
	if a:
		drone.input_yaw = -1.0
		drone.is_landing = false
	elif d:
		drone.input_yaw = 1.0
		drone.is_landing = false
	else:
		drone.input_yaw = 0.0

func _update_camera(delta: float) -> void:
	if not drone or not chase_camera: return
	var d_pos = drone.global_position if is_inside_tree() else drone.position
	var focus_target = d_pos + Vector3(0, 0.12, 0)

	orbit_distance = lerp(orbit_distance, target_orbit_distance, clamp(delta * 14.0, 0.0, 1.0))
	orbit_yaw = lerp_angle(orbit_yaw, target_orbit_yaw, clamp(delta * 16.0, 0.0, 1.0))
	orbit_pitch = lerp(orbit_pitch, target_orbit_pitch, clamp(delta * 16.0, 0.0, 1.0))

	match camera_mode:
		0: # 360 Free Orbit & Macro Inspection
			var cx = orbit_distance * cos(orbit_pitch) * sin(orbit_yaw)
			var cy = orbit_distance * sin(orbit_pitch)
			var cz = orbit_distance * cos(orbit_pitch) * cos(orbit_yaw)
			var desired_pos = focus_target + Vector3(cx, cy, cz)

			if is_inside_tree():
				chase_camera.global_position = desired_pos
				chase_camera.look_at(focus_target, Vector3.UP)
			else:
				chase_camera.position = desired_pos

		1: # Chase Follow Cam
			var rear_offset = drone.global_transform.basis.z * orbit_distance + Vector3(0, orbit_distance * 0.45, 0) if is_inside_tree() else Vector3(0, 1.8, 4.2)
			var target_pos = d_pos + rear_offset
			if is_inside_tree():
				chase_camera.global_position = chase_camera.global_position.lerp(target_pos, delta * 12.0)
				chase_camera.look_at(focus_target, Vector3.UP)

		2: # FPV Cockpit
			if is_inside_tree():
				chase_camera.global_position = d_pos + drone.global_transform.basis.z * -0.42 + Vector3(0, 0.10, 0)
				chase_camera.global_rotation = drone.global_rotation

		3: # Tactical Top-Down
			if is_inside_tree():
				chase_camera.global_position = d_pos + Vector3(0, max(16.0, orbit_distance * 3.0), 0.05)
				chase_camera.look_at(d_pos, Vector3.FORWARD)

func _update_dashboard_telemetry() -> void:
	if not drone or not drone.has_method("get_telemetry"): return
	var t = drone.get_telemetry()

	# 1. Top Bar Status
	if status_badge:
		status_badge.text = "[ " + str(t.flight_mode) + " ]"
		if not t.armed:
			status_badge.modulate = Color(0.7, 0.7, 0.7)
		elif t.flight_mode.contains("RTL"):
			status_badge.modulate = Color(1.0, 0.75, 0.1)
		elif t.flight_mode.contains("ULTRA") or t.power_mode == "ULTRA":
			status_badge.modulate = Color(0.0, 0.9, 1.0)
		else:
			status_badge.modulate = Color(0.0, 1.0, 0.4)

	if battery_label:
		battery_label.text = "⚡ BATT: %.1f%% | %.2fV | %.1fA" % [t.battery_soc_pct, t.battery_volts, t.battery_amps]

	if power_mode_label:
		power_mode_label.text = "POWER: %s (MAX: %d RPM)" % [t.power_mode, int(t.max_rpm_limit)]
		if t.power_mode == "ULTRA":
			power_mode_label.modulate = Color(0.0, 0.9, 1.0)
		elif t.power_mode == "MAX":
			power_mode_label.modulate = Color(1.0, 0.8, 0.2)
		elif t.power_mode == "STOPPED":
			power_mode_label.modulate = Color(0.8, 0.3, 0.3)
		else:
			power_mode_label.modulate = Color(0.3, 1.0, 0.5)

	# 2. Left Avionics Panel
	if alt_value: alt_value.text = "ALT: %.1fm (MSL: %.0fm)" % [t.altitude_agl, t.altitude_msl]
	if alt_target_val: alt_target_val.text = "TGT: %.0fm" % t.target_altitude
	if spd_value: spd_value.text = "SPD: %.1f km/h" % t.ground_speed_kmh
	if vs_value: vs_value.text = "VS: %+.2f m/s" % t.vertical_speed_ms
	if rpy_value: rpy_value.text = "R: %+.1f°  P: %+.1f°  Y: %+.0f°" % [t.roll_deg, t.pitch_deg, t.yaw_deg]

	# 3. Right Motor Bars (M1 - M8)
	var rpms: Array = t.motor_rpms
	for i in range(min(motor_bars.size(), rpms.size())):
		var r = rpms[i]
		motor_bars[i].value = (r / 10000.0) * 100.0
		if i < motor_labels.size():
			motor_labels[i].text = "M%d: %d RPM" % [i + 1, int(r)]
