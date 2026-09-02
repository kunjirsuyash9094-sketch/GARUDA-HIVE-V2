## garuda_model_test.gd
## MASTER MECHANICAL 3D ASSET, GIMBAL, STAND & REAL-LIFE PBR TEXTURE VALIDATION CONTROLLER
## Enforces STEALTH BLACK Default Reference Colorway with Multi-Spectral Optics & Vibration Dampers

extends Node3D

@onready var drone_model = $GARUDA_HL_01
@onready var camera_pivot = $CameraPivot
@onready var camera = $CameraPivot/Camera3D

# Gimbal nodes
var node_gimbal_yaw: Node3D = null
var node_gimbal_pitch: Node3D = null
var node_gimbal_roll: Node3D = null

# 8 Rotor nodes
var rotor_nodes: Array[Node3D] = []
# ROTOR_01=CW, ROTOR_02=CCW, ROTOR_03=CW, ROTOR_04=CCW, ROTOR_05=CW, ROTOR_06=CCW, ROTOR_07=CW, ROTOR_08=CCW
const ROTOR_DIRS = [-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0]

# Rotational Dynamics & Spool Physics (6215 Motor + 16" Carbon Propeller Inertia)
var target_master_rpm: float = 0.0
var actual_motor_rpms = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
const MOTOR_DRAG_FACTORS = [1.00, 0.98, 1.02, 0.99, 1.01, 0.97, 1.03, 1.00]
const SPOOL_UP_RATE: float = 2800.0   # RPM / sec active ESC torque
const COAST_DOWN_RATE: float = 1200.0 # RPM / sec aerodynamic drag & bearing friction

# Camera Orbit & Pan State
var is_orbiting: bool = false
var is_panning: bool = false
var last_mouse_pos: Vector2 = Vector2.ZERO
var cam_yaw: float = 0.0
var cam_pitch: float = -89.5
var cam_distance: float = 1.75
var target_offset: Vector3 = Vector3.ZERO

# UI References
@onready var lbl_span = $UI/PanelMeasurements/LblSpan
@onready var lbl_arm = $UI/PanelMeasurements/LblArm
@onready var lbl_dims = $UI/PanelMeasurements/LblDims
@onready var lbl_variant = $UI/PanelMeasurements/LblVariant
@onready var slider_master_rpm = $UI/PanelControls/SliderMasterRPM
@onready var lbl_rpm_val = $UI/PanelControls/LblRPMVal
@onready var slider_yaw = $UI/PanelControls/SliderGimbalYaw
@onready var slider_pitch = $UI/PanelControls/SliderGimbalPitch
@onready var slider_roll = $UI/PanelControls/SliderGimbalRoll

# Active Color Variant (Default: STEALTH_BLACK)
var current_variant: String = "STEALTH_BLACK"

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
	_init_pbr_materials()
	_bind_model_nodes()
	_setup_measurements()
	set_color_variant("STEALTH_BLACK")
	_on_btn_ortho_view_pressed("TOP")

func _init_pbr_materials() -> void:
	# 1. MAT_STEALTH_CARBON (Primary Fuselage Stealth Armor)
	mat_stealth_carbon = StandardMaterial3D.new()
	mat_stealth_carbon.albedo_color = Color(0.055, 0.058, 0.065, 1.0)
	mat_stealth_carbon.metallic = 0.08
	mat_stealth_carbon.roughness = 0.42
	mat_stealth_carbon.metallic_specular = 0.50
	mat_stealth_carbon.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_stealth_carbon.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	# 2. MAT_CARBON_TUBE (8x Boom Arms & Landing Gear A-Frames & Skids)
	mat_carbon_tube = StandardMaterial3D.new()
	mat_carbon_tube.albedo_color = Color(0.045, 0.048, 0.052, 1.0)
	mat_carbon_tube.metallic = 0.05
	mat_carbon_tube.roughness = 0.35
	mat_carbon_tube.metallic_specular = 0.45
	mat_carbon_tube.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_carbon_tube.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	# 3. MAT_CNC_RED_ALUMINUM (6215 Motor Rings & Damper Bands)
	mat_cnc_red = StandardMaterial3D.new()
	mat_cnc_red.albedo_color = Color(0.78, 0.04, 0.06, 1.0)
	mat_cnc_red.metallic = 0.92
	mat_cnc_red.roughness = 0.22
	mat_cnc_red.metallic_specular = 0.70
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

	# 6. Multi-Spectral Optical Lenses (100% Solid Non-Transparent Real-Life Coatings)
	# 4K Daylight Optical Zoom (Emerald Green Anti-Reflective Coating)
	mat_lens_emerald = StandardMaterial3D.new()
	mat_lens_emerald.albedo_color = Color(0.02, 0.45, 0.28, 1.0)
	mat_lens_emerald.metallic = 0.25
	mat_lens_emerald.roughness = 0.04
	mat_lens_emerald.metallic_specular = 0.95
	mat_lens_emerald.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_lens_emerald.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	# FLIR LWIR Thermal Camera (Deep Purple Germanium Coating)
	mat_lens_germanium = StandardMaterial3D.new()
	mat_lens_germanium.albedo_color = Color(0.38, 0.06, 0.38, 1.0)
	mat_lens_germanium.metallic = 0.30
	mat_lens_germanium.roughness = 0.05
	mat_lens_germanium.metallic_specular = 0.90
	mat_lens_germanium.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_lens_germanium.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	# SWIR / NIR Camera (Sapphire Blue Coating)
	mat_lens_sapphire = StandardMaterial3D.new()
	mat_lens_sapphire.albedo_color = Color(0.06, 0.22, 0.50, 1.0)
	mat_lens_sapphire.metallic = 0.28
	mat_lens_sapphire.roughness = 0.04
	mat_lens_sapphire.metallic_specular = 0.92
	mat_lens_sapphire.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_lens_sapphire.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	# LRF Optical Glass Window
	mat_optical_glass = StandardMaterial3D.new()
	mat_optical_glass.albedo_color = Color(0.04, 0.06, 0.09, 1.0)
	mat_optical_glass.metallic = 0.20
	mat_optical_glass.roughness = 0.04
	mat_optical_glass.metallic_specular = 0.90
	mat_optical_glass.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_optical_glass.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	# 7. MAT_SILICONE_DAMPER (Vibration Isolator Balls)
	mat_silicone_damper = StandardMaterial3D.new()
	mat_silicone_damper.albedo_color = Color(0.55, 0.58, 0.62, 1.0)
	mat_silicone_damper.metallic = 0.02
	mat_silicone_damper.roughness = 0.60
	mat_silicone_damper.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_silicone_damper.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	# 8. MAT_TACTICAL_WHITE (Blade Tip High-Visibility Markings ONLY)
	mat_tactical_white = StandardMaterial3D.new()
	mat_tactical_white.albedo_color = Color(0.90, 0.90, 0.92, 1.0)
	mat_tactical_white.metallic = 0.0
	mat_tactical_white.roughness = 0.25
	mat_tactical_white.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_tactical_white.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	# 9. MAT_PROPELLER_CARBON (16-Inch Cambered Carbon Blade Bodies)
	mat_prop_carbon = StandardMaterial3D.new()
	mat_prop_carbon.albedo_color = Color(0.040, 0.042, 0.045, 1.0)
	mat_prop_carbon.metallic = 0.06
	mat_prop_carbon.roughness = 0.38
	mat_prop_carbon.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_prop_carbon.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	# 10. MAT_DARK_METAL (CNC Arm Sockets, Fasteners, Rails, Antennas)
	mat_dark_metal = StandardMaterial3D.new()
	mat_dark_metal.albedo_color = Color(0.12, 0.13, 0.14, 1.0)
	mat_dark_metal.metallic = 0.88
	mat_dark_metal.roughness = 0.28
	mat_dark_metal.metallic_specular = 0.80
	mat_dark_metal.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_dark_metal.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

	# 11. MAT_MESH_LOUVER (Flank Diamond Mesh Ventilation Grilles)
	mat_mesh_louver = StandardMaterial3D.new()
	mat_mesh_louver.albedo_color = Color(0.025, 0.025, 0.028, 1.0)
	mat_mesh_louver.metallic = 0.35
	mat_mesh_louver.roughness = 0.70
	mat_mesh_louver.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat_mesh_louver.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED

func set_color_variant(variant_name: String) -> void:
	current_variant = variant_name
	if lbl_variant:
		lbl_variant.text = "COLORWAY: " + variant_name.replace("_", " ")

	if not drone_model: return
	_apply_materials_recursive(drone_model)

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

func _bind_model_nodes() -> void:
	if not drone_model:
		print("[!] GARUDA_HL_01 model node not found.")
		return

	# Bind 8 Rotors
	rotor_nodes.clear()
	for i in range(8):
		var num_str = "%02d" % (i + 1)
		var r_node = drone_model.find_child("ROTOR_" + num_str, true, false)
		if r_node:
			rotor_nodes.append(r_node)
		else:
			var fallback = drone_model.find_child("Blade_" + str(i + 1), true, false)
			if fallback:
				rotor_nodes.append(fallback)

	print("[+] Successfully bound %d rotor nodes." % rotor_nodes.size())

	# Bind Gimbal Nodes
	node_gimbal_yaw = drone_model.find_child("GIMBAL_YAW", true, false)
	node_gimbal_pitch = drone_model.find_child("GIMBAL_PITCH", true, false)
	node_gimbal_roll = drone_model.find_child("GIMBAL_ROLL", true, false)

func _setup_measurements() -> void:
	if lbl_span:
		lbl_span.text = "MOTOR SPAN: 1.10 m (Authoritative 8-Rotor Octo-X)"
	if lbl_arm:
		lbl_arm.text = "ARM LENGTH: 0.55 m (Chassis to Motor Shaft)"
	if lbl_dims:
		lbl_dims.text = "FUSELAGE: 0.52m (L) x 0.46m (W) x 0.16m (H)"

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_LEFT:
			is_orbiting = event.pressed
			last_mouse_pos = event.position
		elif event.button_index == MOUSE_BUTTON_RIGHT or event.button_index == MOUSE_BUTTON_MIDDLE:
			is_panning = event.pressed
			last_mouse_pos = event.position
		elif event.button_index == MOUSE_BUTTON_WHEEL_UP:
			cam_distance = clampf(cam_distance - 0.08 * cam_distance, 0.25, 6.0)
			_update_camera_transform()
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			cam_distance = clampf(cam_distance + 0.08 * cam_distance, 0.25, 6.0)
			_update_camera_transform()

	elif event is InputEventMouseMotion:
		var delta = event.position - last_mouse_pos
		last_mouse_pos = event.position
		if is_orbiting:
			cam_yaw -= delta.x * 0.40
			cam_pitch = clampf(cam_pitch - delta.y * 0.30, -89.5, 89.5)
			_update_camera_transform()
		elif is_panning:
			var pan_speed = cam_distance * 0.002
			var right_dir = camera.global_transform.basis.x
			var up_dir = camera.global_transform.basis.y
			target_offset -= right_dir * delta.x * pan_speed - up_dir * delta.y * pan_speed
			_update_camera_transform()

	elif event is InputEventKey and event.pressed:
		if event.keycode == KEY_F:
			_on_btn_ortho_view_pressed("TOP")
		elif event.keycode == KEY_C:
			_on_btn_gimbal_closeup_pressed()

func _update_camera_transform() -> void:
	if not camera_pivot: return
	camera_pivot.position = target_offset
	camera_pivot.rotation_degrees = Vector3(cam_pitch, cam_yaw, 0.0)
	if camera:
		camera.position = Vector3(0, 0, cam_distance)

func _process(delta: float) -> void:
	var avg_actual_rpm: float = 0.0

	# Physical Inertia Integration for 8 Rotors (Spool-Up and Smooth Spin-Down)
	for i in range(rotor_nodes.size()):
		var target_rpm = target_master_rpm
		var curr_rpm = actual_motor_rpms[i]

		if curr_rpm < target_rpm:
			curr_rpm = minf(curr_rpm + SPOOL_UP_RATE * delta, target_rpm)
		elif curr_rpm > target_rpm:
			var aero_drag = (curr_rpm / 3000.0) * (curr_rpm / 3000.0) * 1600.0
			var total_decel = (COAST_DOWN_RATE + aero_drag) * MOTOR_DRAG_FACTORS[i]
			curr_rpm = maxf(curr_rpm - total_decel * delta, target_rpm)

		actual_motor_rpms[i] = curr_rpm
		avg_actual_rpm += curr_rpm

		var r_node = rotor_nodes[i]
		if is_instance_valid(r_node) and curr_rpm > 0.1:
			var rad_s = (curr_rpm / 60.0) * TAU * ROTOR_DIRS[i]
			r_node.rotate_y(rad_s * delta)

	if rotor_nodes.size() > 0:
		avg_actual_rpm /= float(rotor_nodes.size())

	if lbl_rpm_val:
		if absf(avg_actual_rpm - target_master_rpm) > 5.0 and avg_actual_rpm > 1.0:
			lbl_rpm_val.text = "%d RPM (Spooling)" % int(avg_actual_rpm)
		else:
			lbl_rpm_val.text = "%d RPM" % int(target_master_rpm)

# =============================================================================
# UI Callbacks
# =============================================================================
func _on_slider_master_rpm_value_changed(value: float) -> void:
	target_master_rpm = value

func set_preset_rpm(rpm_val: float) -> void:
	target_master_rpm = rpm_val
	if slider_master_rpm:
		slider_master_rpm.value = rpm_val

func _on_slider_gimbal_yaw_value_changed(value: float) -> void:
	if is_instance_valid(node_gimbal_yaw):
		node_gimbal_yaw.rotation_degrees.y = value

func _on_slider_gimbal_pitch_value_changed(value: float) -> void:
	if is_instance_valid(node_gimbal_pitch):
		node_gimbal_pitch.rotation_degrees.x = value

func _on_slider_gimbal_roll_value_changed(value: float) -> void:
	if is_instance_valid(node_gimbal_roll):
		node_gimbal_roll.rotation_degrees.z = value

func _on_btn_gimbal_closeup_pressed() -> void:
	target_offset = Vector3(0.0, -0.18, -0.12)
	cam_yaw = 22.0
	cam_pitch = -8.0
	cam_distance = 0.55
	_update_camera_transform()

func _on_btn_stand_closeup_pressed() -> void:
	target_offset = Vector3(0.0, -0.25, 0.0)
	cam_yaw = 45.0
	cam_pitch = -12.0
	cam_distance = 0.95
	_update_camera_transform()

func _on_btn_hero_front_pressed() -> void:
	target_offset = Vector3.ZERO
	cam_yaw = 32.0
	cam_pitch = -16.0
	cam_distance = 1.65
	_update_camera_transform()

func _on_btn_ortho_view_pressed(view_mode: String) -> void:
	target_offset = Vector3.ZERO
	match view_mode:
		"TOP":
			cam_yaw = 0.0
			cam_pitch = -89.5
			cam_distance = 1.75
		"HERO":
			cam_yaw = 32.0
			cam_pitch = -16.0
			cam_distance = 1.65
		"FRONT":
			cam_yaw = 0.0
			cam_pitch = 0.0
			cam_distance = 1.65
		"BACK":
			cam_yaw = 180.0
			cam_pitch = 0.0
			cam_distance = 1.65
		"LEFT":
			cam_yaw = -90.0
			cam_pitch = 0.0
			cam_distance = 1.65
		"RIGHT":
			cam_yaw = 90.0
			cam_pitch = 0.0
			cam_distance = 1.65
		"BOTTOM":
			cam_yaw = 0.0
			cam_pitch = 89.5
			cam_distance = 1.75
		"FRONT_45":
			cam_yaw = 45.0
			cam_pitch = -18.0
			cam_distance = 1.65
		"REAR_45":
			cam_yaw = 135.0
			cam_pitch = -18.0
			cam_distance = 1.65
	_update_camera_transform()
