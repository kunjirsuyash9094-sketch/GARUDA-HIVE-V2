## input_setup.gd  (autoload singleton "InputSetup")
## Registers the drone control actions in code so the demo works out of the box
## with no manual Project → Input Map configuration. Keyboard control also has a
## direct-key fallback inside drone_controller.gd; these actions add gamepad
## support and satisfy the arm/disarm action lookups.

extends Node

func _enter_tree() -> void:
	# Arm / disarm
	_add_key("drone_arm",    KEY_ENTER)
	_add_joy_button("drone_arm",    JOY_BUTTON_START)
	_add_key("drone_disarm", KEY_BACKSPACE)
	_add_joy_button("drone_disarm", JOY_BUTTON_B)

	# Flight axes (gamepad). Keyboard is handled by drone_controller.gd's fallback.
	_add_axis("drone_throttle_pos", JOY_AXIS_LEFT_Y,  -1.0)  # stick up  = more throttle
	_add_axis("drone_throttle_neg", JOY_AXIS_LEFT_Y,   1.0)
	_add_axis("drone_yaw_pos",      JOY_AXIS_LEFT_X,   1.0)
	_add_axis("drone_yaw_neg",      JOY_AXIS_LEFT_X,  -1.0)
	_add_axis("drone_pitch_pos",    JOY_AXIS_RIGHT_Y, -1.0)
	_add_axis("drone_pitch_neg",    JOY_AXIS_RIGHT_Y,  1.0)
	_add_axis("drone_roll_pos",     JOY_AXIS_RIGHT_X,  1.0)
	_add_axis("drone_roll_neg",     JOY_AXIS_RIGHT_X, -1.0)

func _ensure(action: String) -> void:
	if not InputMap.has_action(action):
		InputMap.add_action(action)

func _add_key(action: String, keycode: Key) -> void:
	_ensure(action)
	var e := InputEventKey.new()
	e.physical_keycode = keycode
	InputMap.action_add_event(action, e)

func _add_joy_button(action: String, button: JoyButton) -> void:
	_ensure(action)
	var e := InputEventJoypadButton.new()
	e.button_index = button
	InputMap.action_add_event(action, e)

func _add_axis(action: String, axis: JoyAxis, value: float) -> void:
	_ensure(action)
	var e := InputEventJoypadMotion.new()
	e.axis = axis
	e.axis_value = value
	InputMap.action_add_event(action, e)
