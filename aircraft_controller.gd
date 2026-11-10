class_name AircraftController
# Faithful 1:1 port of AircraftController.cs from lunetis/OperationZERO (Unity).
# Every formula, constant, and quirk preserved — including the Autopilot y-wrap bug.
# Original: https://github.com/lunetis/OperationZERO Assets/Scripts/AircraftController.cs
#
# Units are theirs: speed in m/s (UI shows speed*10), altitude in world units
# (UI shows *5). Unity fixedDeltaTime = 0.02 s.
#
# Unity->Godot convention mapping (Godot: +Y up, forward = -Z, right-handed):
#   Unity Euler(x,y,z) [x=nose-down+, y=yaw-right+, z=roll-right+] maps to
#   q = Quaternion(UP,-y) * Quaternion(RIGHT,-x) * Quaternion(Z,-z)   (verified by round-trip test)

const FIXED_DT := 0.02  # Unity default fixedDeltaTime

# --- serialized values taken from ZERO.unity player block (real scene values) ---
var maxSpeed := 250.0
var minSpeed := 15.0
var defaultSpeed := 60.0
var throttleAmount := 5.0
var accelAmount := 25.0
var brakeAmount := 100.0
var calibrateAmount := 50.0
var rollAmount := 125.0
var pitchAmount := 50.0
var yawAmount := 30.0
var rotateLerpAmount := 4.0
var highGFactor := 1.2
var highGTurnTime := 2.0
var stallSpeed := 25.0
var stallHeight := 4000.0
var gravityFactor := 5.0

# --- inputs (Unity input action values, 0..1) ---
var accelValue := 0.0
var brakeValue := 0.0
var rollValue := 0.0     # their convention: +1 -> rotateVector.z negative
var pitchValue := 0.0    # their convention: negative = pull up
var yawRValue := 0.0
var yawLValue := 0.0

# --- state ---
var speed := 60.0
var throttle := 0.0
var q := Quaternion()                      # attitude
var position_units := Vector3.ZERO   # transform.position equivalent
var rotateValue := Vector3.ZERO      # smoothed body rates deg/s (x pitch, y yaw, z roll)
var isStalling := false
var isAutoPilot := false
var isHighGTurning := false

var _highGCooldown := 2.0
var _highGReciprocal := 0.5
var _isHighGPressed := false
var _isHighGEnabled := true
var _speedReciprocal := 1.0 / 250.0
var prev_position_units := Vector3.ZERO   # state before last step (for render interpolation)
var prev_q := Quaternion()

func reset() -> void:
	accelValue = 0.0; brakeValue = 0.0; rollValue = 0.0
	pitchValue = 0.0; yawRValue = 0.0; yawLValue = 0.0
	_highGCooldown = highGTurnTime
	_highGReciprocal = 1.0 / _highGCooldown
	_isHighGPressed = false
	_isHighGEnabled = true
	_speedReciprocal = 1.0 / maxSpeed
	speed = defaultSpeed
	throttle = 0.0
	q = Quaternion()
	position_units = Vector3.ZERO
	prev_position_units = Vector3.ZERO
	prev_q = Quaternion()
	rotateValue = Vector3.ZERO
	isStalling = false; isAutoPilot = false; isHighGTurning = false

# --- Unity convention helpers ---
func u_euler(x: float, y: float, z: float) -> Quaternion:
	# Unity Quaternion.Euler(x,y,z): applied z, then x, then y (left-handed)
	return Quaternion(Vector3.UP, -deg_to_rad(y)) * Quaternion(Vector3.RIGHT, -deg_to_rad(x)) \
		* Quaternion(Vector3(0, 0, 1), -deg_to_rad(z))

func forward() -> Vector3:
	return q * Vector3.FORWARD

func _wrap360(a: float) -> float:
	a = fmod(a, 360.0)
	if a < 0.0:
		a += 360.0
	return a

func quat_to_unity_euler(rq: Quaternion) -> Vector3:
	# Unity transform.eulerAngles equivalent: YXZ extraction, [0,360).
	# q = Y(-y) X(-x) Z(-z) in right-handed terms, so negate extracted (psi,theta,phi).
	# NOTE: Godot Basis[i] indexes COLUMNS: matrix m[r][c] == b[c][r].
	var b := Basis(rq)
	var t := rad_to_deg(asin(clamp(-b[2][1], -1.0, 1.0)))
	var y := rad_to_deg(atan2(b[2][0], b[2][2]))
	var z := rad_to_deg(atan2(b[0][1], b[1][1]))
	return Vector3(_wrap360(-t), _wrap360(-y), _wrap360(-z))

func unity_euler() -> Vector3:
	return quat_to_unity_euler(q)

# --- MoveAircraft() ---
func step() -> void:
	prev_position_units = position_units
	prev_q = q
	var accel := accelValue
	var brake := brakeValue
	var highGPitchFactor := 1.0

	# CheckHighGTurn (original mutates accel/brake/factor by ref)
	isHighGTurning = false
	if accelValue == 1.0 and brakeValue == 1.0:
		if pitchValue < -0.7:
			if _isHighGEnabled:
				_isHighGEnabled = false
				_isHighGPressed = true
			if _highGCooldown < 0.0:
				_isHighGPressed = false
			if _isHighGEnabled or _isHighGPressed:
				accel = 0.0
				brake = brakeValue * highGFactor * (1.0 + _highGCooldown * _highGReciprocal)
				highGPitchFactor = highGFactor * (1.0 + _highGCooldown * _highGReciprocal)
				_highGCooldown -= FIXED_DT
				isHighGTurning = true
	else:
		_isHighGPressed = false
		_isHighGEnabled = true
	if not _isHighGPressed:
		_highGCooldown += FIXED_DT * 2.0
		if _highGCooldown >= highGTurnTime:
			_highGCooldown = highGTurnTime

	var eu := unity_euler()
	var rotateVector := Vector3.ZERO

	# === Rotation ===
	if speed < stallSpeed or position_units.y > stallHeight:
		# Ignore all rotation input and head to the ground
		isStalling = true
		var target := u_euler(90.0, eu.y, eu.z)
		var diff := q.inverse() * target
		var diffAngle := quat_to_unity_euler(diff)
		# Adjustment (original)
		if diffAngle.x > 180.0: diffAngle.x -= 360.0
		if diffAngle.y > 180.0: diffAngle.y -= 360.0
		if diffAngle.z > 180.0: diffAngle.z -= 360.0
		diffAngle.x = clampf(diffAngle.x, -pitchAmount, pitchAmount)
		diffAngle.y = clampf(diffAngle.y, -yawAmount, yawAmount)
		diffAngle.z = clampf(diffAngle.z, -rollAmount, rollAmount)
		rotateValue = rotateValue.lerp(diffAngle, rotateLerpAmount * FIXED_DT)
	else:
		isStalling = false
		if yawLValue == 1.0 and yawRValue == 1.0:
			isAutoPilot = true
			rotateVector = -eu
			if rotateVector.x < -180.0: rotateVector.x += 360.0
			if rotateVector.z < -180.0: rotateVector.y += 360.0  # original bug, kept
			if rotateVector.z < -180.0: rotateVector.z += 360.0
			rotateVector.x = clampf(rotateVector.x * 2.0, -pitchAmount, pitchAmount)
			rotateVector.z = clampf(rotateVector.z * 2.0, -rollAmount, rollAmount)
			rotateVector.y = 0.0
		else:
			isAutoPilot = false
			rotateVector = Vector3(pitchValue * pitchAmount * highGPitchFactor,
				(yawRValue - yawLValue) * yawAmount, -rollValue * rollAmount)
		rotateValue = rotateValue.lerp(rotateVector, rotateLerpAmount * FIXED_DT)

	q = (q * u_euler(rotateValue.x * FIXED_DT, rotateValue.y * FIXED_DT, rotateValue.z * FIXED_DT)).normalized()

	# === Move ===
	throttle = lerp(throttle, accel - brake, throttleAmount * FIXED_DT)

	if throttle > 0.0:
		var accelEase := (maxSpeed + (position_units.y * 0.01) - speed) * _speedReciprocal
		speed += throttle * accelAmount * accelEase * FIXED_DT
	elif throttle < 0.0:
		var brakeEase := (speed - minSpeed) * _speedReciprocal
		speed += throttle * brakeAmount * brakeEase * FIXED_DT

	var release := 1.0 - absf(throttle)
	speed += release * (defaultSpeed - speed) * _speedReciprocal * calibrateAmount * FIXED_DT

	# Gravity
	speed += gravityFactor * sin(deg_to_rad(eu.x)) * FIXED_DT

	position_units += forward() * speed * FIXED_DT

# --- UI equivalents ---
func display_speed() -> int:
	return int(speed * 10.0)

func display_altitude() -> int:
	return int(position_units.y * 5.0)

func display_heading() -> float:
	return unity_euler().y
