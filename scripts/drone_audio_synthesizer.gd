## drone_audio_synthesizer.gd
## Real-Time Procedural Multirotor Drone Audio Engine (Brushless Motors + Propeller Aerodynamics + HUD Chimes)
## Uses Godot 4 AudioStreamGenerator (Zero external asset dependencies, 100% standalone)

extends Node
class_name DroneAudioSynthesizer

@export var drone: Node

var motor_player: AudioStreamPlayer
var motor_generator: AudioStreamGenerator
var motor_playback: AudioStreamGeneratorPlayback

var chime_player: AudioStreamPlayer
var chime_generator: AudioStreamGenerator
var chime_playback: AudioStreamGeneratorPlayback

# Synthesis parameters
const SAMPLE_RATE: float = 44100.0
var phase_m1: float = 0.0
var phase_m2: float = 0.0
var phase_sub: float = 0.0
var phase_noise: float = 0.0

# Chime queue
var chime_freq: float = 0.0
var chime_duration: float = 0.0
var chime_time: float = 0.0
var chime_phase: float = 0.0

func _ready() -> void:
	_setup_motor_audio()
	_setup_chime_audio()

func _setup_motor_audio() -> void:
	motor_player = AudioStreamPlayer.new()
	motor_generator = AudioStreamGenerator.new()
	motor_generator.mix_rate = SAMPLE_RATE
	motor_generator.buffer_length = 0.1
	motor_player.stream = motor_generator
	motor_player.volume_db = -6.0
	add_child(motor_player)
	motor_player.play()
	motor_playback = motor_player.get_stream_playback()

func _setup_chime_audio() -> void:
	chime_player = AudioStreamPlayer.new()
	chime_generator = AudioStreamGenerator.new()
	chime_generator.mix_rate = SAMPLE_RATE
	chime_generator.buffer_length = 0.1
	chime_player.stream = chime_generator
	chime_player.volume_db = -4.0
	add_child(chime_player)
	chime_player.play()
	chime_playback = chime_player.get_stream_playback()

func _process(_delta: float) -> void:
	_fill_motor_audio_buffer()
	_fill_chime_audio_buffer()

func _fill_motor_audio_buffer() -> void:
	if not motor_playback: return

	var frames_available = motor_playback.get_frames_available()
	if frames_available <= 0: return

	var avg_rpm = 0.0
	if drone and "motor_rpms" in drone:
		for r in drone.motor_rpms: avg_rpm += r
		avg_rpm /= 8.0

	var norm_rpm = clamp(avg_rpm / 5500.0, 0.0, 1.0)
	var motor_vol = norm_rpm * 0.35 if (drone and "armed" in drone and drone.armed) else 0.0

	# Fundamental blade pass frequency
	var base_freq = 65.0 + norm_rpm * 260.0
	var harm_freq = base_freq * 2.15
	var sub_freq = base_freq * 0.5

	for i in range(frames_available):
		if motor_vol <= 0.001:
			motor_playback.push_frame(Vector2.ZERO)
			continue

		phase_m1 += (base_freq / SAMPLE_RATE) * TAU
		phase_m2 += (harm_freq / SAMPLE_RATE) * TAU
		phase_sub += (sub_freq / SAMPLE_RATE) * TAU

		if phase_m1 > TAU: phase_m1 -= TAU
		if phase_m2 > TAU: phase_m2 -= TAU
		if phase_sub > TAU: phase_sub -= TAU

		var s1 = sin(phase_m1) * 0.45
		var s2 = sin(phase_m2) * 0.25
		var s3 = 0.2 if sin(phase_sub) > 0.0 else -0.2
		var noise = (randf() * 2.0 - 1.0) * (0.15 * norm_rpm)

		var sample = (s1 + s2 + s3 + noise) * motor_vol
		motor_playback.push_frame(Vector2(sample, sample))

func _fill_chime_audio_buffer() -> void:
	if not chime_playback: return

	var frames_available = chime_playback.get_frames_available()
	if frames_available <= 0: return

	var dt_sample = 1.0 / SAMPLE_RATE
	for i in range(frames_available):
		if chime_time < chime_duration and chime_freq > 0:
			chime_time += dt_sample
			chime_phase += (chime_freq / SAMPLE_RATE) * TAU
			if chime_phase > TAU: chime_phase -= TAU
			var env = clamp(1.0 - (chime_time / chime_duration), 0.0, 1.0)
			var val = sin(chime_phase) * env * 0.4
			chime_playback.push_frame(Vector2(val, val))
		else:
			chime_playback.push_frame(Vector2.ZERO)

# Tactical HUD Beep Chimes
func play_arm_chime() -> void:
	chime_freq = 880.0 # A5
	chime_duration = 0.22
	chime_time = 0.0
	chime_phase = 0.0

func play_takeoff_chime() -> void:
	chime_freq = 1174.66 # D6
	chime_duration = 0.35
	chime_time = 0.0
	chime_phase = 0.0

func play_land_chime() -> void:
	chime_freq = 659.25 # E5
	chime_duration = 0.40
	chime_time = 0.0
	chime_phase = 0.0

func play_touchdown_chime() -> void:
	chime_freq = 523.25 # C5
	chime_duration = 0.50
	chime_time = 0.0
	chime_phase = 0.0
