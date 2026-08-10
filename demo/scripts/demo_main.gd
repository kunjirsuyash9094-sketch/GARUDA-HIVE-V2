## demo_main.gd
## Headless live-demo driver: arms the ArduPilot JSON bridge and prints
## telemetry + SITL link status to stdout so the pipeline can be watched
## from the console while an external probe drives the bridge.
extends Node3D

@onready var drone: DroneBody = $DroneBody
@onready var sitl: SITLManager = $DroneBody/SITLManager

const RUN_SECONDS := 20.0
const PRINT_INTERVAL := 0.5

var _elapsed := 0.0
var _since_print := 0.0

func _ready() -> void:
	print("[demo] SkySim SITL live demo starting")
	print("[demo] physics_ticks_per_second=%d" % Engine.physics_ticks_per_second)
	sitl.enabled_ardupilot = true
	sitl.port_ardupilot = 9002
	print("[demo] ArduPilot JSON bridge listening on UDP :9002")
	print("[demo] waiting for servo packets...")

func _physics_process(delta: float) -> void:
	_elapsed += delta
	_since_print += delta
	if _since_print >= PRINT_INTERVAL:
		_since_print = 0.0
		_report()
	if _elapsed >= RUN_SECONDS:
		print("[demo] run time elapsed, shutting down")
		get_tree().quit()

func _report() -> void:
	var telem: Dictionary = drone.get_telemetry()
	var status: Dictionary = sitl.get_sitl_status()
	var ap: Dictionary = status.get("ardupilot", {})
	print("[t=%5.1fs] sitl_active=%-5s source=%-9s | alt=%6.2fm  vs=%+5.2fm/s  thrust=%6.1fN | AP connected=%-5s last_rx=%sms" % [
		_elapsed,
		telem.get("sitl_active", false),
		telem.get("sitl_source", "none"),
		telem.get("altitude", 0.0),
		telem.get("vertical_speed", 0.0),
		telem.get("total_thrust", 0.0),
		ap.get("connected", false),
		ap.get("last_rx_ms", -1),
	])
