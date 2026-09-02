## drone_physics_engine.gd
## High-Performance 6-DOF Bulletproof Multirotor Flight Engine
## Built for GARUDA-HL-01 Cinema Octocopter in Godot 4

extends Node3D
class_name DronePhysicsEngine

# =============================================================================
# 1. Physical Specifications
# =============================================================================
@export var airframe_mass: float = 8.50
@export var payload_mass: float = 2.20
@export var arm_length: float = 0.55
@export var rotor_radius: float = 0.28
@export var landing_gear_height: float = 0.38

const MAX_RPM: float = 5500.0
const IDLE_RPM: float = 1200.0
const HOVER_RPM: float = 3350.0

# 8 Rotors (Octo-X alternating directions)
var rotor_angles: Array[float] = [
	deg_to_rad(22.5),  deg_to_rad(67.5),  deg_to_rad(112.5), deg_to_rad(157.5),
	deg_to_rad(202.5), deg_to_rad(247.5), deg_to_rad(292.5), deg_to_rad(337.5)
]
var rotor_dir: Array[float] = [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0]

# =============================================================================
# 2. Flight States & Variables
# =============================================================================
var armed: bool = false
var flight_mode: String = "READY // PRESS W OR SPACE"
var is_landing: bool = false
var target_altitude: float = 0.38
var current_yaw_rad: float = 0.0
var velocity: Vector3 = Vector3.ZERO
var home_pos: Vector3 = Vector3(0.0, 0.38, 0.0)

# Pilot input setpoints
var input_climb: float = 0.0    # -1 to +1
var input_forward: float = 0.0  # -1 to +1
var input_strafe: float = 0.0   # -1 to +1
var input_yaw: float = 0.0      # -1 to +1

# Motor Telemetry
var motor_rpms: Array[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
var motor_targets: Array[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Battery Model (22,000 mAh 6S)
var battery_soc: float = 1.0
var battery_v_term: float = 25.20
var battery_current: float = 0.0
var battery_mah_consumed: float = 0.0
const BATTERY_CAPACITY_MAH: float = 22000.0

# Visual Mesh Blades & Audio
@onready var visual_rotor_blades: Array[Node3D] = []
var audio_node: DroneAudioSynthesizer = null

func _ready() -> void:
	Engine.physics_ticks_per_second = 400
	_find_rotor_blades()
	current_yaw_rad = rotation.y
	audio_node = get_tree().root.find_child("DroneAudio", true, false)

func _find_rotor_blades() -> void:
	visual_rotor_blades.clear()
	for i in range(8):
		var blade = find_child("Blade_%d" % (i + 1), true, false)
		if blade:
			visual_rotor_blades.append(blade)

# =============================================================================
# 3. 400Hz Flight Physics Loop
# =============================================================================
func _physics_process(dt: float) -> void:
	var cur_pos = global_position if is_inside_tree() else position
	var agl = max(0.0, cur_pos.y - landing_gear_height)

	# 1. DISARMED
	if not armed:
		flight_mode = "DISARMED"
		velocity = Vector3.ZERO
		is_landing = false
		for i in range(8):
			motor_targets[i] = 0.0
			motor_rpms[i] = lerp(motor_rpms[i], 0.0, dt * 15.0)
		cur_pos.y = landing_gear_height
		rotation.x = 0.0
		rotation.z = 0.0
		_set_pos(cur_pos)
		_update_battery(dt)
		_animate_rotors(dt)
		return

	# 2. RTL AUTO-LANDING
	if is_landing:
		_process_rtl(dt, agl, cur_pos)
	else:
		# Manual Flight with Dynamic Altitude Hold
		if abs(input_climb) > 0.05:
			velocity.y = lerp(velocity.y, input_climb * 4.5, dt * 8.0)
			target_altitude = max(0.38, cur_pos.y - landing_gear_height)
			flight_mode = "CLIMBING" if input_climb > 0 else "DESCENDING"
		else:
			# Auto-Hover at target altitude
			var alt_err = target_altitude - agl
			velocity.y = lerp(velocity.y, clamp(alt_err * 3.5, -3.0, 4.0), dt * 10.0)
			flight_mode = "AUTO-HOVER" if agl > 0.2 else "ARMED ON GROUND"

		# Horizontal Flight Control
		current_yaw_rad += input_yaw * 2.2 * dt

		var fwd_spd = input_forward * 9.5
		var str_spd = input_strafe * 9.5

		var cos_y = cos(current_yaw_rad)
		var sin_y = sin(current_yaw_rad)
		var target_vx = str_spd * cos_y - fwd_spd * sin_y
		var target_vz = str_spd * sin_y + fwd_spd * cos_y

		velocity.x = lerp(velocity.x, target_vx, dt * 8.0)
		velocity.z = lerp(velocity.z, target_vz, dt * 8.0)

		if abs(input_forward) > 0.1 or abs(input_strafe) > 0.1:
			flight_mode = "MANUAL FLIGHT"

	# 3. DYNAMIC ATTITUDE VISUAL TILT
	var body_fwd_vel = -velocity.x * sin(current_yaw_rad) + velocity.z * cos(current_yaw_rad)
	var body_str_vel = velocity.x * cos(current_yaw_rad) + velocity.z * sin(current_yaw_rad)

	var tilt_pitch = clamp(-body_fwd_vel * 2.2, -22.0, 22.0)
	var tilt_roll = clamp(body_str_vel * 2.2, -22.0, 22.0)

	var target_basis = Basis.from_euler(Vector3(deg_to_rad(tilt_pitch), current_yaw_rad, deg_to_rad(tilt_roll)))
	if is_inside_tree():
		global_transform.basis = global_transform.basis.slerp(target_basis, clamp(dt * 14.0, 0.0, 1.0)).orthonormalized()
	else:
		transform.basis = target_basis

	# 4. TRANSLATIONAL POSITION UPDATE
	cur_pos += velocity * dt

	# 5. GROUND CONTACT (Only clamp when falling downward)
	if cur_pos.y <= landing_gear_height:
		if velocity.y <= 0.0:
			cur_pos.y = landing_gear_height
			velocity.y = 0.0
			velocity.x *= 0.5
			velocity.z *= 0.5
			if is_landing and agl <= 0.03:
				armed = false
				is_landing = false
				flight_mode = "DOCKED ON PAD"
				cur_pos = home_pos
				if is_inside_tree():
					global_transform.basis = Basis.from_euler(Vector3(0, current_yaw_rad, 0))
				if audio_node and audio_node.has_method("play_touchdown_chime"):
					audio_node.play_touchdown_chime()

	_set_pos(cur_pos)

	# 6. MOTOR RPM SYNTHESIS
	var base_rpm = HOVER_RPM if agl > 0.1 else IDLE_RPM
	if velocity.y > 0.5: base_rpm = HOVER_RPM + 850.0
	elif velocity.y < -0.5: base_rpm = HOVER_RPM - 650.0

	for i in range(8):
		var ang = rotor_angles[i]
		var diff = -sin(ang) * tilt_roll * 10.0 + cos(ang) * tilt_pitch * 10.0 + rotor_dir[i] * input_yaw * 350.0
		motor_targets[i] = clamp(base_rpm + diff, IDLE_RPM, MAX_RPM)
		motor_rpms[i] = lerp(motor_rpms[i], motor_targets[i], dt * 25.0)

	_update_battery(dt)
	_animate_rotors(dt)

func _set_pos(p: Vector3) -> void:
	if is_inside_tree():
		global_position = p
	else:
		position = p

# =============================================================================
# 4. RTL Navigation
# =============================================================================
func _process_rtl(dt: float, agl: float, cur_pos: Vector3) -> void:
	flight_mode = "RTL AUTO-LAND"
	var to_home = home_pos - cur_pos
	var dist_horiz = Vector2(to_home.x, to_home.z).length()

	if dist_horiz > 0.35:
		var nav_dir = Vector2(to_home.x, to_home.z).normalized()
		var spd = clamp(dist_horiz * 2.0, 1.5, 6.5)
		velocity.x = lerp(velocity.x, nav_dir.x * spd, dt * 6.0)
		velocity.z = lerp(velocity.z, nav_dir.y * spd, dt * 6.0)

		var alt_target = max(3.0, target_altitude)
		var alt_err = alt_target - agl
		velocity.y = lerp(velocity.y, clamp(alt_err * 3.0, -2.5, 3.5), dt * 8.0)
	else:
		velocity.x = lerp(velocity.x, 0.0, dt * 8.0)
		velocity.z = lerp(velocity.z, 0.0, dt * 8.0)
		velocity.y = lerp(velocity.y, -0.65, dt * 6.0)

# =============================================================================
# 5. Battery & Rotor Animation
# =============================================================================
func _update_battery(dt: float) -> void:
	if not armed:
		battery_current = 0.6
	else:
		var sum_rpm = 0.0
		for r in motor_rpms: sum_rpm += r
		var pwr_w = (sum_rpm / (8.0 * 5000.0)) * 1450.0
		battery_current = pwr_w / battery_v_term

	var mah_used = (battery_current * 1000.0) * (dt / 3600.0)
	battery_mah_consumed += mah_used
	battery_soc = clamp(1.0 - (battery_mah_consumed / BATTERY_CAPACITY_MAH), 0.0, 1.0)
	battery_v_term = 19.8 + battery_soc * 5.4 - battery_current * 0.012

func _animate_rotors(dt: float) -> void:
	for i in range(min(visual_rotor_blades.size(), motor_rpms.size())):
		var blade = visual_rotor_blades[i]
		if blade:
			var rad_s = (motor_rpms[i] / 60.0) * TAU * rotor_dir[i]
			blade.rotate_y(rad_s * dt)

# =============================================================================
# 6. Interactive Command Actions
# =============================================================================
func trigger_arm_takeoff() -> void:
	armed = true
	is_landing = false
	target_altitude = 2.5
	if audio_node and audio_node.has_method("play_arm_chime"):
		audio_node.play_arm_chime()

func trigger_disarm() -> void:
	armed = false
	is_landing = false
	flight_mode = "DISARMED"

func trigger_auto_hover() -> void:
	armed = true
	is_landing = false
	input_forward = 0.0
	input_strafe = 0.0
	input_climb = 0.0
	input_yaw = 0.0
	var cur_y = global_position.y if is_inside_tree() else position.y
	target_altitude = max(1.5, cur_y - landing_gear_height)
	flight_mode = "AUTO-HOVER"

func trigger_rtl_landing() -> void:
	if armed:
		is_landing = true
		if audio_node and audio_node.has_method("play_land_chime"):
			audio_node.play_land_chime()

func reset_to_pad() -> void:
	armed = false
	is_landing = false
	_set_pos(home_pos)
	velocity = Vector3.ZERO
	current_yaw_rad = 0.0
	if is_inside_tree():
		global_transform.basis = Basis.IDENTITY
	else:
		transform.basis = Basis.IDENTITY
	target_altitude = 0.38
	flight_mode = "READY // PRESS W OR SPACE"

func get_telemetry() -> Dictionary:
	var cur_pos = global_position if is_inside_tree() else position
	var cur_rot = global_rotation if is_inside_tree() else rotation
	var agl = max(0.0, cur_pos.y - landing_gear_height)
	return {
		"armed": armed,
		"flight_mode": flight_mode,
		"position": cur_pos,
		"velocity": velocity,
		"altitude_agl": agl,
		"target_altitude": target_altitude,
		"ground_speed_kmh": Vector2(velocity.x, velocity.z).length() * 3.6,
		"vertical_speed_ms": velocity.y,
		"roll_deg": rad_to_deg(cur_rot.z),
		"pitch_deg": rad_to_deg(cur_rot.x),
		"yaw_deg": rad_to_deg(cur_rot.y),
		"battery_soc_pct": battery_soc * 100.0,
		"battery_volts": battery_v_term,
		"battery_amps": battery_current,
		"motor_rpms": motor_rpms
	}
