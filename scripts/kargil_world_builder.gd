## kargil_world_builder.gd
## High-Fidelity Studio & Testing Arena with PBR Lighting & SSAO for Drone Inspection

extends Node3D
class_name KargilWorldBuilder

func _ready() -> void:
	_build_arena_ground()
	_build_tactical_launchpad()
	_build_approach_corridor_markers()
	_setup_studio_atmosphere()

# =============================================================================
# 1. Plain Flat Tarmac Testing Ground
# =============================================================================
func _build_arena_ground() -> void:
	var ground_mesh = PlaneMesh.new()
	ground_mesh.size = Vector2(400.0, 400.0)

	var ground_inst = MeshInstance3D.new()
	ground_inst.mesh = ground_mesh

	var mat = StandardMaterial3D.new()
	mat.albedo_color = Color(0.10, 0.11, 0.13)
	mat.roughness = 0.85
	mat.metallic = 0.20
	ground_inst.material_override = mat

	var body = StaticBody3D.new()
	var col = CollisionShape3D.new()
	var shape = WorldBoundaryShape3D.new()
	col.shape = shape
	body.add_child(col)
	ground_inst.add_child(body)

	add_child(ground_inst)

# =============================================================================
# 2. Tactical Helipad / Launchpad LZ-01
# =============================================================================
func _build_tactical_launchpad() -> void:
	var pad_group = Node3D.new()
	pad_group.name = "Launchpad_LZ01"
	pad_group.position = Vector3(0, 0.01, 0)

	# 1. Base Concrete Slab
	var base_mesh = CylinderMesh.new()
	base_mesh.top_radius = 3.2
	base_mesh.bottom_radius = 3.2
	base_mesh.height = 0.04
	base_mesh.radial_segments = 48

	var base_inst = MeshInstance3D.new()
	base_inst.mesh = base_mesh
	var base_mat = StandardMaterial3D.new()
	base_mat.albedo_color = Color(0.18, 0.20, 0.23)
	base_mat.roughness = 0.70
	base_inst.material_override = base_mat
	pad_group.add_child(base_inst)

	# 2. Safety Orange Perimeter Rim
	var rim_mesh = TorusMesh.new()
	rim_mesh.inner_radius = 3.0
	rim_mesh.outer_radius = 3.2
	rim_mesh.rings = 48
	rim_mesh.ring_segments = 24

	var rim_inst = MeshInstance3D.new()
	rim_inst.mesh = rim_mesh
	rim_inst.position.y = 0.03
	var rim_mat = StandardMaterial3D.new()
	rim_mat.albedo_color = Color(0.96, 0.48, 0.08)
	rim_mat.roughness = 0.4
	rim_inst.material_override = rim_mat
	pad_group.add_child(rim_inst)

	# 3. Tactical 'H' Center Markings
	var h_left = BoxMesh.new()
	h_left.size = Vector3(0.35, 0.02, 2.2)
	var h_inst1 = MeshInstance3D.new()
	h_inst1.mesh = h_left
	h_inst1.position = Vector3(-0.75, 0.03, 0)

	var h_inst2 = MeshInstance3D.new()
	h_inst2.mesh = h_left
	h_inst2.position = Vector3(0.75, 0.03, 0)

	var h_bar = BoxMesh.new()
	h_bar.size = Vector3(1.5, 0.02, 0.35)
	var h_inst3 = MeshInstance3D.new()
	h_inst3.mesh = h_bar
	h_inst3.position = Vector3(0, 0.03, 0)

	var h_mat = StandardMaterial3D.new()
	h_mat.albedo_color = Color(0.95, 0.95, 0.95)
	h_mat.roughness = 0.3
	h_inst1.material_override = h_mat
	h_inst2.material_override = h_mat
	h_inst3.material_override = h_mat

	pad_group.add_child(h_inst1)
	pad_group.add_child(h_inst2)
	pad_group.add_child(h_inst3)

	# 4. 4 Corner Green LED Beacon Towers
	var beacon_mesh = CylinderMesh.new()
	beacon_mesh.top_radius = 0.08
	beacon_mesh.bottom_radius = 0.08
	beacon_mesh.height = 0.28

	var beacon_mat = StandardMaterial3D.new()
	beacon_mat.albedo_color = Color(0.0, 1.0, 0.4)
	beacon_mat.emission_enabled = true
	beacon_mat.emission = Color(0.0, 1.0, 0.4)
	beacon_mat.emission_energy_multiplier = 3.0

	var offsets = [
		Vector3(2.5, 0.14, 2.5),
		Vector3(-2.5, 0.14, 2.5),
		Vector3(2.5, 0.14, -2.5),
		Vector3(-2.5, 0.14, -2.5)
	]
	for off in offsets:
		var b = MeshInstance3D.new()
		b.mesh = beacon_mesh
		b.material_override = beacon_mat
		b.position = off
		pad_group.add_child(b)

		var light = OmniLight3D.new()
		light.light_color = Color(0.0, 1.0, 0.4)
		light.light_energy = 1.0
		light.omni_range = 3.0
		light.position = off + Vector3(0, 0.15, 0)
		pad_group.add_child(light)

	add_child(pad_group)

# =============================================================================
# 3. Tactical Approach Corridor & Guide Lights
# =============================================================================
func _build_approach_corridor_markers() -> void:
	var corridor_group = Node3D.new()
	corridor_group.name = "ApproachCorridor"

	var pylon_mesh = CylinderMesh.new()
	pylon_mesh.top_radius = 0.05
	pylon_mesh.bottom_radius = 0.05
	pylon_mesh.height = 0.18

	var amber_mat = StandardMaterial3D.new()
	amber_mat.albedo_color = Color(1.0, 0.70, 0.10)
	amber_mat.emission_enabled = true
	amber_mat.emission = Color(1.0, 0.70, 0.10)
	amber_mat.emission_energy_multiplier = 2.5

	var z_positions = [4.5, 7.0, 9.5, 12.0]
	for z_pos in z_positions:
		for x_pos in [-1.8, 1.8]:
			var p = MeshInstance3D.new()
			p.mesh = pylon_mesh
			p.material_override = amber_mat
			p.position = Vector3(x_pos, 0.09, z_pos)
			corridor_group.add_child(p)

	add_child(corridor_group)

# =============================================================================
# 4. Cinematic Lighting & PBR Studio Atmosphere
# =============================================================================
func _setup_studio_atmosphere() -> void:
	var env = Environment.new()
	env.background_mode = Environment.BG_SKY

	var sky = Sky.new()
	var sky_mat = ProceduralSkyMaterial.new()
	sky_mat.sky_top_color = Color(0.35, 0.58, 0.85)
	sky_mat.sky_horizon_color = Color(0.70, 0.80, 0.90)
	sky_mat.ground_bottom_color = Color(0.18, 0.20, 0.24)
	sky.sky_material = sky_mat
	env.sky = sky

	# High-Fidelity Tonemapping & Post-Processing
	env.tonemap_mode = Environment.TONE_MAPPER_ACES
	env.glow_enabled = true
	env.glow_intensity = 0.6
	env.glow_bloom = 0.20
	env.ssao_enabled = true
	env.ssao_radius = 1.5
	env.ssao_intensity = 1.8

	var world_env = WorldEnvironment.new()
	world_env.environment = env
	add_child(world_env)

	# Key Sun Light
	var sun = DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-45, 40, 0)
	sun.light_color = Color(1.0, 0.98, 0.94)
	sun.light_energy = 1.35
	sun.shadow_enabled = true
	sun.shadow_bias = 0.02
	sun.shadow_blur = 1.2
	add_child(sun)

	# Atmospheric Fill Light
	var fill = DirectionalLight3D.new()
	fill.rotation_degrees = Vector3(-30, -140, 0)
	fill.light_color = Color(0.65, 0.75, 0.90)
	fill.light_energy = 0.50
	add_child(fill)
