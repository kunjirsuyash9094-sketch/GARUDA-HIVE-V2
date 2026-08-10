## chase_camera.gd
## A smooth follow camera. Tracks the target's POSITION (not its roll/pitch),
## so the horizon stays stable and the drone never flips the view around.
## Attach to a Camera3D and assign `target` to the DroneBody.

extends Camera3D

@export var target: Node3D
@export var target_path: NodePath
## World-space offset from the target: behind (+Z) and above (+Y).
@export var offset: Vector3 = Vector3(0.0, 4.0, 11.0)
## Higher = snappier follow, lower = lazier/cinematic.
@export_range(0.5, 20.0, 0.5) var follow_speed: float = 4.0
## Point the camera slightly above the drone so it sits in the lower third.
@export var look_height: float = 1.0

func _ready() -> void:
	if target == null and not target_path.is_empty():
		target = get_node_or_null(target_path)
	current = true
	if target:
		global_position = target.global_position + offset

func _physics_process(delta: float) -> void:
	if target == null:
		return
	var desired: Vector3 = target.global_position + offset
	var t: float = 1.0 - exp(-follow_speed * delta)   # framerate-independent lerp
	global_position = global_position.lerp(desired, t)
	look_at(target.global_position + Vector3(0.0, look_height, 0.0), Vector3.UP)
