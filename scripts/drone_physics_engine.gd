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

const MAX_RPM: float = 8000.0
const IDLE_RPM: float = 1200.0
const HOVER_RPM_BASE: float = 3100.0

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

# Battery Model (22,000 mAh 6S High-C LiPo)
var battery_soc: float = 1.0
var battery_v_term: float = 25.20
var battery_current: float = 0.0
var battery_mah_consumed: float = 0.0
const BATTERY_CAPACITY_MAH: float = 22000.0

# Visual Mesh Blades & Audio
@onready var visual_rotor_blades: Array[Node3D] = []
var audio_node: DroneAudioSynthesizer = null

# Real-Life High-Quality PBR Materials
var mat_stealth_carbon: StandardMaterial3D
var mat_carbon_tube: StandardMaterial3D
var mat_cnc_red: StandardMaterial3D
var mat_cyan_status: StandardMaterial3D
var mat_red_status: StandardMaterial3D
var mat_optical_glass: StandardMaterial3D
var mat_lens_emerald: StandardMaterial3D
var mat_lens_germanium: StandardMaterial3D
var mat_lens_sapphire: StandardMaterial3D
var mat_silicone_damper: StandardMaterial3D
var mat_tactical_white: StandardMaterial3D
var mat_prop_carbon: StandardMaterial3D
var mat_dark_metal: StandardMaterial3D
var mat_mesh_louver: StandardMaterial3D

func _ready() -> void:
	Engine.physics_ticks_per_second = 400
	_init_pbr_materials()
	_apply_materials_recursive(self)
	_find_rotor_blades()
	current_yaw_rad = rotation.y
	audio_node = get_tree().root.find_child("DroneAudio", true, false)

func _init_pbr_materials() -> void:
	mat_stealth_carbon = StandardMaterial3D.new()
	mat_stealth_carbon.albedo_color = Color(0.055, 0.058, 0.065, 1.0)
	mat_stealth_carbon.metallic = 0.08
	mat_stealth_carbon.roughness = 0.42

	mat_carbon_tube = StandardMaterial3D.new()
	mat_carbon_tube.albedo_color = Color(0.045, 0.048, 0.052, 1.0)
	mat_carbon_tube.metallic = 0.05
	mat_carbon_tube.roughness = 0.35

	# 1. MAT_STEALTH_CARBON (Primary Fuselage Stealth Armor)
	mat_stealth_carbon = StandardMaterial3D.new()
	mat_stealth_carbon.albedo_color = Color(0.055, 0.058, 0.065, 1.0)
	mat_stealth_carbon.metallic = 0.08
	mat_stealth_carbon.roughness = 0.42
	mat_stealth_carbon.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_stealth_carbon.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	# 2. MAT_CARBON_TUBE (8x Boom Arms & Landing Gear A-Frames & Skids)
	mat_carbon_tube = StandardMaterial3D.new()
	mat_carbon_tube.albedo_color = Color(0.045, 0.048, 0.052, 1.0)
	mat_carbon_tube.metallic = 0.05
	mat_carbon_tube.roughness = 0.35
	mat_carbon_tube.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_carbon_tube.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	# 3. MAT_CNC_RED_ALUMINUM (6215 Motor Rings & Damper Bands)
	mat_cnc_red = StandardMaterial3D.new()
	mat_cnc_red.albedo_color = Color(0.78, 0.04, 0.06, 1.0)
	mat_cnc_red.metallic = 0.92
	mat_cnc_red.roughness = 0.22
	mat_cnc_red.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_cnc_red.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	# 4. MAT_CYAN_STATUS (Recessed Lightguides & Arm Stripes #00E5FF)
	mat_cyan_status = StandardMaterial3D.new()
	mat_cyan_status.albedo_color = Color(0.0, 0.90, 1.0, 1.0)
	mat_cyan_status.emission_enabled = true
	mat_cyan_status.emission = Color(0.0, 0.90, 1.0, 1.0)
	mat_cyan_status.emission_energy_multiplier = 3.5
	mat_cyan_status.cull_mode = BaseMaterial3D.CULL_DISABLED

	# 5. MAT_RED_STATUS (Aviation Anti-Collision Warning Beacon)
	mat_red_status = StandardMaterial3D.new()
	mat_red_status.albedo_color = Color(1.0, 0.04, 0.04, 1.0)
	mat_red_status.emission_enabled = true
	mat_red_status.emission = Color(1.0, 0.04, 0.04, 1.0)
	mat_red_status.emission_energy_multiplier = 4.0
	mat_red_status.cull_mode = BaseMaterial3D.CULL_DISABLED

	# 6. Multi-Spectral Optical Lenses (100% Solid Non-Transparent AR Coated)
	mat_lens_emerald = StandardMaterial3D.new()
	mat_lens_emerald.albedo_color = Color(0.02, 0.45, 0.28, 1.0)
	mat_lens_emerald.metallic = 0.25
	mat_lens_emerald.roughness = 0.04
	mat_lens_emerald.metallic_specular = 0.95
	mat_lens_emerald.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_lens_emerald.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	mat_lens_germanium = StandardMaterial3D.new()
	mat_lens_germanium.albedo_color = Color(0.38, 0.06, 0.38, 1.0)
	mat_lens_germanium.metallic = 0.30
	mat_lens_germanium.roughness = 0.05
	mat_lens_germanium.metallic_specular = 0.90
	mat_lens_germanium.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_lens_germanium.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	mat_lens_sapphire = StandardMaterial3D.new()
	mat_lens_sapphire.albedo_color = Color(0.06, 0.22, 0.50, 1.0)
	mat_lens_sapphire.metallic = 0.28
	mat_lens_sapphire.roughness = 0.04
	mat_lens_sapphire.metallic_specular = 0.92
	mat_lens_sapphire.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_lens_sapphire.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	mat_optical_glass = StandardMaterial3D.new()
	mat_optical_glass.albedo_color = Color(0.04, 0.06, 0.09, 1.0)
	mat_optical_glass.metallic = 0.20
	mat_optical_glass.roughness = 0.04
	mat_optical_glass.metallic_specular = 0.90
	mat_optical_glass.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_optical_glass.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	mat_silicone_damper = StandardMaterial3D.new()
	mat_silicone_damper.albedo_color = Color(0.55, 0.58, 0.62, 1.0)
	mat_silicone_damper.metallic = 0.02
	mat_silicone_damper.roughness = 0.60
	mat_silicone_damper.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_silicone_damper.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	mat_tactical_white = StandardMaterial3D.new()
	mat_tactical_white.albedo_color = Color(0.90, 0.90, 0.92, 1.0)
	mat_tactical_white.metallic = 0.0
	mat_tactical_white.roughness = 0.25
	mat_tactical_white.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_tactical_white.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	mat_prop_carbon = StandardMaterial3D.new()
	mat_prop_carbon.albedo_color = Color(0.040, 0.042, 0.045, 1.0)
	mat_prop_carbon.metallic = 0.06
	mat_prop_carbon.roughness = 0.38
	mat_prop_carbon.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_prop_carbon.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	mat_dark_metal = StandardMaterial3D.new()
	mat_dark_metal.albedo_color = Color(0.12, 0.13, 0.14, 1.0)
	mat_dark_metal.metallic = 0.88
	mat_dark_metal.roughness = 0.28
	mat_dark_metal.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_dark_metal.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	mat_mesh_louver = StandardMaterial3D.new()
	mat_mesh_louver.albedo_color = Color(0.025, 0.025, 0.028, 1.0)
	mat_mesh_louver.metallic = 0.35
	mat_mesh_louver.roughness = 0.70
	mat_mesh_louver.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_mesh_louver.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

func _apply_materials_recursive(node: Node) -> void:
	if node is MeshInstance3D and node.mesh:
		var n_name = node.name.to_upper()
		for surf_idx in range(node.mesh.get_surface_count()):
			var target_mat: Material = null
			if n_name.contains("CYAN") or n_name.contains("GLOW") or n_name.contains("DETAILS"):
				target_mat = mat_cyan_status
			elif n_name.contains("BEACON") or n_name.contains("RED_STATUS"):
				target_mat = mat_red_status
			elif n_name.contains("RED") or n_name.contains("MOTOR_RING") or n_name.contains("MOUNT_RINGS"):
				target_mat = mat_cnc_red
			elif n_name.contains("DAMPER"):
				target_mat = mat_silicone_damper
			elif n_name.contains("EMERALD") or n_name.contains("DAYLIGHT"):
				target_mat = mat_lens_emerald
			elif n_name.contains("GERMANIUM") or n_name.contains("THERMAL"):
				target_mat = mat_lens_germanium
			elif n_name.contains("SAPPHIRE") or n_name.contains("SWIR"):
				target_mat = mat_lens_sapphire
			elif n_name.contains("LRF") or n_name.contains("GLASS"):
				target_mat = mat_optical_glass
			elif n_name.contains("TIP") or n_name.contains("WHITE"):
				target_mat = mat_tactical_white
			elif n_name.contains("BLADE") or n_name.contains("PROPELLER"):
				target_mat = mat_prop_carbon
			elif n_name.contains("LOUVER") or n_name.contains("MESH"):
				target_mat = mat_mesh_louver
			elif n_name.contains("TUBE") or n_name.contains("ARM_CARBON") or n_name.contains("GEAR"):
				target_mat = mat_carbon_tube
			elif n_name.contains("SOCKET") or n_name.contains("COLLAR") or n_name.contains("CAN") or n_name.contains("ANTENNA") or n_name.contains("HUB") or n_name.contains("MOUNT") or n_name.contains("GIMBAL") or n_name.contains("METAL") or n_name.contains("BEZEL"):
				target_mat = mat_dark_metal
			else:
				target_mat = mat_stealth_carbon

			if target_mat:
				node.set_surface_override_material(surf_idx, target_mat)

	for child in node.get_children():
		_apply_materials_recursive(child)

func _find_rotor_blades() -> void:
	visual_rotor_blades.clear()
	for i in range(8):
		var num_str = "%02d" % (i + 1)
		var blade = find_child("ROTOR_" + num_str, true, false)
		if not blade:
			blade = find_child("Blade_%d" % (i + 1), true, false)
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
		# Manual Flight with Dynamic Altitude Hold (High Power Climb up to 4000m+)
		if abs(input_climb) > 0.05:
			var max_climb_rate = 14.0 if input_climb > 0 else 8.0
			velocity.y = lerp(velocity.y, input_climb * max_climb_rate, dt * 10.0)
			target_altitude = max(0.38, cur_pos.y - landing_gear_height)
			flight_mode = "CLIMBING (HIGH POWER)" if input_climb > 0 else "DESCENDING"
		else:
			# Auto-Hover at target altitude with high-altitude altitude-gain compensation
			var alt_err = target_altitude - agl
			velocity.y = lerp(velocity.y, clamp(alt_err * 4.5, -6.0, 8.0), dt * 12.0)
			flight_mode = "AUTO-HOVER" if agl > 0.2 else "ARMED ON GROUND"

		# High-Speed Horizontal Flight Control (Up to 28 m/s = 100+ km/h)
		current_yaw_rad += input_yaw * 3.5 * dt

		var fwd_spd = input_forward * 28.0
		var str_spd = input_strafe * 28.0

		var cos_y = cos(current_yaw_rad)
		var sin_y = sin(current_yaw_rad)
		var target_vx = str_spd * cos_y - fwd_spd * sin_y
		var target_vz = str_spd * sin_y + fwd_spd * cos_y

		velocity.x = lerp(velocity.x, target_vx, dt * 10.0)
		velocity.z = lerp(velocity.z, target_vz, dt * 10.0)

		if abs(input_forward) > 0.1 or abs(input_strafe) > 0.1:
			flight_mode = "HIGH-SPEED CRUISE (100+ KM/H)"

	# 3. DYNAMIC ATTITUDE VISUAL TILT
	var body_fwd_vel = -velocity.x * sin(current_yaw_rad) + velocity.z * cos(current_yaw_rad)
	var body_str_vel = velocity.x * cos(current_yaw_rad) + velocity.z * sin(current_yaw_rad)

	var tilt_pitch = clamp(-body_fwd_vel * 1.5, -28.0, 28.0)
	var tilt_roll = clamp(body_str_vel * 1.5, -28.0, 28.0)

	var target_basis = Basis.from_euler(Vector3(deg_to_rad(tilt_pitch), current_yaw_rad, deg_to_rad(tilt_roll)))
	if is_inside_tree():
		global_transform.basis = global_transform.basis.slerp(target_basis, clamp(dt * 16.0, 0.0, 1.0)).orthonormalized()
	else:
		transform.basis = target_basis

	# 4. TRANSLATIONAL POSITION UPDATE
	cur_pos += velocity * dt

	# 5. GROUND CONTACT (Only clamp when falling downward)
	if cur_pos.y <= landing_gear_height:
		if velocity.y <= 0.0:
			cur_pos.y = landing_gear_height
			velocity.y = 0.0
			velocity.x *= 0.4
			velocity.z *= 0.4
			if is_landing and agl <= 0.04:
				armed = false
				is_landing = false
				flight_mode = "DOCKED ON PAD"
				cur_pos = home_pos
				if is_inside_tree():
					global_transform.basis = Basis.from_euler(Vector3(0, current_yaw_rad, 0))
				if audio_node and audio_node.has_method("play_touchdown_chime"):
					audio_node.play_touchdown_chime()

	_set_pos(cur_pos)

	# 6. MOTOR RPM SYNTHESIS with 8000 RPM Max & High-Altitude Barometric Compensation (4000m+)
	var altitude_msl = 3100.0 + agl # Kargil High-Altitude Base MSL
	var density_ratio = clamp(exp(-altitude_msl / 8500.0) / 0.693, 0.45, 1.0)
	var dynamic_hover_rpm = HOVER_RPM_BASE / sqrt(density_ratio) # Increases RPM as air thins out at 4000m+

	var base_rpm = dynamic_hover_rpm if agl > 0.1 else IDLE_RPM
	if velocity.y > 0.5: base_rpm = dynamic_hover_rpm + velocity.y * 220.0
	elif velocity.y < -0.5: base_rpm = max(IDLE_RPM, dynamic_hover_rpm + velocity.y * 180.0)

	var horiz_boost = Vector2(velocity.x, velocity.z).length() * 55.0
	base_rpm += horiz_boost

	for i in range(8):
		var ang = rotor_angles[i]
		var diff = -sin(ang) * tilt_roll * 14.0 + cos(ang) * tilt_pitch * 14.0 + rotor_dir[i] * input_yaw * 500.0
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
# 4. Fast & Crisp RTL Navigation
# =============================================================================
func _process_rtl(dt: float, agl: float, cur_pos: Vector3) -> void:
	flight_mode = "RTL AUTO-LAND"
	var to_home = home_pos - cur_pos
	var dist_horiz = Vector2(to_home.x, to_home.z).length()

	if dist_horiz > 0.25:
		var nav_dir = Vector2(to_home.x, to_home.z).normalized()
		var spd = clamp(dist_horiz * 3.5, 3.0, 16.0) # Fast transit home
		velocity.x = lerp(velocity.x, nav_dir.x * spd, dt * 8.0)
		velocity.z = lerp(velocity.z, nav_dir.y * spd, dt * 8.0)

		var alt_target = max(3.5, target_altitude)
		var alt_err = alt_target - agl
		velocity.y = lerp(velocity.y, clamp(alt_err * 4.0, -3.0, 5.0), dt * 10.0)
	else:
		# Rapid descent with touchdown flare
		velocity.x = lerp(velocity.x, 0.0, dt * 12.0)
		velocity.z = lerp(velocity.z, 0.0, dt * 12.0)
		if agl > 0.8:
			velocity.y = lerp(velocity.y, -2.4, dt * 8.0) # Fast descent
		else:
			velocity.y = lerp(velocity.y, -0.75, dt * 10.0) # Touchdown flare

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
