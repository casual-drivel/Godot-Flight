# ============================================================================
# OperationZERO flight model — faithful Godot port
# ============================================================================
# Source: AircraftController.cs from lunetis/OperationZERO (MIT — see
# LICENSE.OperationZERO.txt). Every formula, constant, and quirk matches the
# Unity original; the tuning values below were read out of the player block
# in the ZERO.unity mission scene, not guessed from code defaults.
#
# Naming: the C# camelCase fields became snake_case (accelValue -> accel_value,
# maxSpeed -> max_speed, ...). Behavior is unchanged; parity_test.gd proves it.
#
# Units (the original's): speed in m/s (HUD shows x10), altitude in world
# units (HUD shows x5). Fixed timestep 0.02 s = Unity's fixedDeltaTime.
#
# Coordinate mapping — Godot is right-handed with forward = -Z, Unity is
# left-handed with forward = +Z. A Unity Euler(x, y, z)
# [x = nose-down+, y = yaw-right+, z = roll-right+] maps to:
#     q = Quaternion(UP, -y) * Quaternion(RIGHT, -x) * Quaternion(Z, -z)
# This mapping is verified by a 500-sample round-trip in parity_test.gd
# (max error 0.0001 deg).
# ============================================================================

const FIXED_DT := 0.02  # Unity default fixedDeltaTime

# --- Tuning (serialized values from ZERO.unity, player AircraftController) ---
var max_speed := 250.0        # m/s (HUD: 2500)
var min_speed := 15.0
var default_speed := 60.0     # "neutral" speed the throttle calibration drifts to
var throttle_amount := 5.0    # lerp rate of the throttle toward accel - brake
var accel_amount := 25.0      # m/s^2 scale when throttle > 0
var brake_amount := 100.0     # m/s^2 scale when throttle < 0
var calibrate_amount := 50.0  # pull toward default_speed when throttle ~centered
var roll_amount := 125.0      # deg/s at full roll input
var pitch_amount := 50.0      # deg/s at full pitch input
var yaw_amount := 30.0        # deg/s at full rudder input
var rotate_lerp_amount := 4.0 # smoothing rate of the rotation rates
var high_g_factor := 1.2      # high-G turn strength multiplier
var high_g_turn_time := 2.0   # seconds of high-G turn in the budget
var stall_speed := 25.0       # m/s below which the jet departs
var stall_height := 4000.0    # world units above which the jet departs
var gravity_factor := 5.0     # m/s^2 exchanged with climb/dive via sin(pitch)

# --- Inputs (Unity input action values, 0..1) ---
var accel_value := 0.0
var brake_value := 0.0
var roll_value := 0.0    # original sign: +1 rolls LEFT (rotate.z = -roll * amount)
var pitch_value := 0.0   # original sign: negative = pull up
var yaw_r_value := 0.0
var yaw_l_value := 0.0

# --- State ---
var speed := 60.0                    # m/s along the nose (rail model: no sideslip)
var throttle := 0.0                  # smoothed accel - brake, in [-1, 1]
var attitude := Quaternion()         # orientation (see mapping note above)
var position_units := Vector3.ZERO   # transform.position equivalent
var rotate_value := Vector3.ZERO     # smoothed body rates deg/s (x pitch, y yaw, z roll)
var is_stalling := false
var is_autopilot := false
var is_high_g_turning := false
var prev_position_units := Vector3.ZERO  # state before last step (render interpolation)
var prev_attitude := Quaternion()

# --- High-G turn internals (C#: highGCooldown / highGReciprocal / isHighG*) ---
var _hg_cooldown := 2.0
var _hg_reciprocal := 0.5
var _hg_pressed := false   # latches while the accel+brake combo is held
var _hg_enabled := true    # one-shot: consumed until the combo is released
var _speed_reciprocal := 1.0 / 250.0


func reset() -> void:
	accel_value = 0.0; brake_value = 0.0; roll_value = 0.0
	pitch_value = 0.0; yaw_r_value = 0.0; yaw_l_value = 0.0
	_hg_cooldown = high_g_turn_time
	_hg_reciprocal = 1.0 / _hg_cooldown
	_hg_pressed = false
	_hg_enabled = true
	_speed_reciprocal = 1.0 / max_speed
	speed = default_speed
	throttle = 0.0
	attitude = Quaternion()
	position_units = Vector3.ZERO
	prev_position_units = Vector3.ZERO
	prev_attitude = Quaternion()
	rotate_value = Vector3.ZERO
	is_stalling = false; is_autopilot = false; is_high_g_turning = false


# ==========================================================================
# Unity-convention math helpers
# ==========================================================================

## Build the rotation a Unity Quaternion.Euler(x, y, z) would produce,
## expressed in Godot space (see mapping note at the top).
func u_euler(x: float, y: float, z: float) -> Quaternion:
	return Quaternion(Vector3.UP, -deg_to_rad(y)) \
		* Quaternion(Vector3.RIGHT, -deg_to_rad(x)) \
		* Quaternion(Vector3(0, 0, 1), -deg_to_rad(z))

## Unit forward vector (Unity's transform.forward).
func forward() -> Vector3:
	return attitude * Vector3.FORWARD

func _wrap360(a: float) -> float:
	a = fmod(a, 360.0)
	if a < 0.0:
		a += 360.0
	return a

## Unity transform.eulerAngles equivalent: YXZ extraction, [0, 360).
## attitude = Y(-y) X(-x) Z(-z) in right-handed terms, so the extracted
## (yaw, pitch, roll) are negated. NOTE: Godot Basis[i] indexes COLUMNS,
## i.e. matrix m[row][col] == b[col][row].
func quat_to_unity_euler(rq: Quaternion) -> Vector3:
	var b := Basis(rq)
	var pitch := rad_to_deg(asin(clamp(-b[2][1], -1.0, 1.0)))
	var yaw := rad_to_deg(atan2(b[2][0], b[2][2]))
	var roll := rad_to_deg(atan2(b[0][1], b[1][1]))
	return Vector3(_wrap360(-pitch), _wrap360(-yaw), _wrap360(-roll))

func unity_euler() -> Vector3:
	return quat_to_unity_euler(attitude)


# ==========================================================================
# Main step (C#: MoveAircraft, called from FixedUpdate)
# ==========================================================================

func step() -> void:
	prev_position_units = position_units
	prev_attitude = attitude

	# C# passes accel/brake/pitch_factor by ref into CheckHighGTurn;
	# here the helper returns the modified copies.
	var hg := _check_high_g_turn()
	var accel: float = hg[0]
	var brake: float = hg[1]
	var pitch_factor: float = hg[2]

	# Attitude snapshot for this step (the original computes eulerAngles
	# once up front and reuses it for stall/autopilot/gravity).
	var eu := unity_euler()

	# --- Rotation: stall overrides everything, else autopilot or raw input ---
	var rotate_target := Vector3.ZERO
	if speed < stall_speed or position_units.y > stall_height:
		# Departure: ignore all input and point the nose at the ground.
		is_stalling = true
		rotate_target = _stall_correction(eu)
		rotate_value = rotate_value.lerp(rotate_target, rotate_lerp_amount * FIXED_DT)
	else:
		is_stalling = false
		if yaw_l_value == 1.0 and yaw_r_value == 1.0:
			# Autopilot: both rudders held (the original's gamepad shoulders).
			is_autopilot = true
			rotate_target = _autopilot_target(eu)
		else:
			is_autopilot = false
			rotate_target = Vector3(pitch_value * pitch_amount * pitch_factor,
				(yaw_r_value - yaw_l_value) * yaw_amount, -roll_value * roll_amount)
		rotate_value = rotate_value.lerp(rotate_target, rotate_lerp_amount * FIXED_DT)

	# Body-frame incremental rotation: q' = q * Euler(rates * dt)
	attitude = (attitude * u_euler(rotate_value.x * FIXED_DT,
		rotate_value.y * FIXED_DT, rotate_value.z * FIXED_DT)).normalized()

	_update_speed(accel, brake, eu.x)
	position_units += forward() * speed * FIXED_DT


# ==========================================================================
# Sub-systems
# ==========================================================================

## High-G turn (C#: CheckHighGTurn). Trigger: accel AND brake held AND a hard
## pull (pitch_value < -0.7). While firing: thrust off, braking and pitch rate
## scaled by high_g_factor * (1 + cooldown/turn_time) — so the boost decays
## from 2x down to 1x as the 2 s budget drains. One-shot per keypress; the
## budget recharges at 2x while the combo is not firing.
## Returns [accel, brake, pitch_factor] after modification.
func _check_high_g_turn() -> Array:
	var accel := accel_value
	var brake := brake_value
	var pitch_factor := 1.0
	is_high_g_turning = false

	if accel_value == 1.0 and brake_value == 1.0:
		if pitch_value < -0.7:
			if _hg_enabled:
				_hg_enabled = false   # consume the one-shot for this keypress
				_hg_pressed = true
			if _hg_cooldown < 0.0:
				_hg_pressed = false   # budget exhausted (it may go negative)
			if _hg_enabled or _hg_pressed:
				accel = 0.0
				var scale := high_g_factor * (1.0 + _hg_cooldown * _hg_reciprocal)
				brake = brake_value * scale
				pitch_factor = scale
				_hg_cooldown -= FIXED_DT
				is_high_g_turning = true
	else:
		_hg_pressed = false
		_hg_enabled = true   # rearm for the next keypress

	if not _hg_pressed:
		_hg_cooldown = minf(_hg_cooldown + FIXED_DT * 2.0, high_g_turn_time)

	return [accel, brake, pitch_factor]


## Stall recovery (C#: Stall()). Steer the nose straight down (Unity x = 90)
## while keeping yaw and roll, using the smallest per-axis correction clamped
## to the control rates. The resulting dive rebuilds speed until
## speed > stall_speed.
func _stall_correction(eu: Vector3) -> Vector3:
	var target := u_euler(90.0, eu.y, eu.z)
	var diff := attitude.inverse() * target
	var da := quat_to_unity_euler(diff)
	# Wrap to (-180, 180] exactly like the original's adjustment block.
	if da.x > 180.0: da.x -= 360.0
	if da.y > 180.0: da.y -= 360.0
	if da.z > 180.0: da.z -= 360.0
	da.x = clampf(da.x, -pitch_amount, pitch_amount)
	da.y = clampf(da.y, -yaw_amount, yaw_amount)
	da.z = clampf(da.z, -roll_amount, roll_amount)
	return da


## Autopilot level-off (C#: Autopilot()). Drives pitch and roll to level at
## up to 2 deg/s per deg of error, clamped to the control rates; yaw is left
## alone. NOTE: the `rv.y += 360` under the z-check is a bug in the original
## — kept verbatim for fidelity.
func _autopilot_target(eu: Vector3) -> Vector3:
	var rv := -eu
	if rv.x < -180.0: rv.x += 360.0
	if rv.z < -180.0: rv.y += 360.0  # original bug, kept
	if rv.z < -180.0: rv.z += 360.0
	rv.x = clampf(rv.x * 2.0, -pitch_amount, pitch_amount)
	rv.z = clampf(rv.z * 2.0, -roll_amount, roll_amount)
	rv.y = 0.0
	return rv


## Speed integration (C#: the Move half of MoveAircraft). Three terms:
##  1. throttle easing — accel eases toward max_speed (slightly higher at
##     altitude), braking eases toward min_speed; both fade near the target
##  2. calibration — with the lever centered, speed drifts toward default_speed
##  3. gravity — climbing bleeds speed, diving gains it (sin of pitch)
## pitch_deg is the step's attitude snapshot (the original reuses the
## eulerAngles computed before the rotation update).
## Quirk: after a stall the nose points down, so gravity can carry the jet
## past default_speed during recovery — that is the original's behavior.
func _update_speed(accel: float, brake: float, pitch_deg: float) -> void:
	throttle = lerp(throttle, accel - brake, throttle_amount * FIXED_DT)

	if throttle > 0.0:
		var accel_ease := (max_speed + (position_units.y * 0.01) - speed) * _speed_reciprocal
		speed += throttle * accel_amount * accel_ease * FIXED_DT
	elif throttle < 0.0:
		var brake_ease := (speed - min_speed) * _speed_reciprocal
		speed += throttle * brake_amount * brake_ease * FIXED_DT

	var release := 1.0 - absf(throttle)
	speed += release * (default_speed - speed) * _speed_reciprocal * calibrate_amount * FIXED_DT

	speed += gravity_factor * sin(deg_to_rad(pitch_deg)) * FIXED_DT


# ==========================================================================
# UI equivalents (C#: SetUI multipliers)
# ==========================================================================

func display_speed() -> int:
	return int(speed * 10.0)

func display_altitude() -> int:
	return int(position_units.y * 5.0)

func display_heading() -> float:
	return unity_euler().y
