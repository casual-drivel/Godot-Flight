extends SceneTree
# Side B parity test: verifies the Unity->Godot math mapping and runs the
# OperationZERO model through the same scenarios as Side A for A/B comparison.

func _initialize() -> void:
	var AC = load("res://aircraft_controller.gd").new()

	# 1) Quaternion mapping round-trip: u_euler -> extract must be identity
	var rng := RandomNumberGenerator.new()
	rng.seed = 42
	var max_err := 0.0
	for i in 500:
		var x := rng.randf_range(-85.0, 85.0)  # YXZ pitch domain (asin), like Unity
		var y := rng.randf_range(-170.0, 170.0)
		var z := rng.randf_range(-170.0, 170.0)
		if false:
			continue
		var e = AC.quat_to_unity_euler(AC.u_euler(x, y, z))
		var ex := fmod(e.x - x + 540.0, 360.0) - 180.0
		var ey := fmod(e.y - y + 540.0, 360.0) - 180.0
		var ez := fmod(e.z - z + 540.0, 360.0) - 180.0
		max_err = maxf(max_err, maxf(absf(ex), maxf(absf(ey), absf(ez))))
	print("ROUNDTRIP: max_err=%.4f deg (expect <0.01)" % max_err)

	# sanity: x=90 must point forward straight down (their stall target)
	AC.reset(); AC.q = AC.u_euler(90.0, 0.0, 0.0)
	print("PITCH90_FORWARD: %s (expect ~ (0,-1,0))" % AC.forward())
	AC.reset(); AC.q = AC.u_euler(0.0, 90.0, 0.0)
	print("YAW90_FORWARD: %s (expect ~ (1,0,0) = right)" % AC.forward())

	# 2) AB ramp from default speed (accel held 30 s)
	AC.reset(); AC.accelValue = 1.0
	var marks := {}
	for t in range(int(30.0 / AC.FIXED_DT)):
		AC.step()
		var now = (t + 1) * AC.FIXED_DT
		for m in [5.0, 10.0, 20.0, 30.0]:
			if absf(now - m) < AC.FIXED_DT:
				marks[m] = AC.speed
	print("AB_RAMP: t5=%.1f t10=%.1f t20=%.1f t30=%.1f m/s (max %.0f, display %.0f)" % [
		marks[5.0], marks[10.0], marks[20.0], marks[30.0], AC.speed, AC.display_speed()])

	# 3) Full braking from top speed (10 s)
	for t in range(int(10.0 / AC.FIXED_DT)):
		AC.accelValue = 0.0; AC.brakeValue = 1.0
		AC.step()
	print("BRAKE10: speed=%.1f m/s (ease toward min %.0f)" % [AC.speed, AC.minSpeed])

	# 4) Release: calibration drift toward defaultSpeed (from above stall speed,
	#    level attitude — note: in test 3 the jet actually stalled and dove,
	#    which is exactly what OperationZERO does; this isolates calibration).
	AC.reset(); AC.speed = 100.0; AC.position_units = Vector3(0, 500, 0)
	AC.accelValue = 0.0; AC.brakeValue = 0.0
	for t in range(int(10.0 / AC.FIXED_DT)):
		AC.step()
	print("CALIBRATE10: speed=%.1f (drifting toward %.0f)" % [AC.speed, AC.defaultSpeed])

	# 5) Stall: force speed below stallSpeed -> nose must swing to ground
	AC.reset(); AC.speed = 20.0; AC.position_units = Vector3(0, 500, 0)
	AC.step()
	print("STALL: isStalling=%s (expect true, speed %.1f < %.0f)" % [AC.isStalling, AC.speed, AC.stallSpeed])
	for t in range(int(3.0 / AC.FIXED_DT)):
		AC.step()
	var e = AC.unity_euler()
	print("STALL3S: pitch=%.0f speed=%.0f (nose swung down, gravity recovered speed)" % [e.x, AC.speed])

	# 6) Ceiling stall: fast but above stallHeight
	AC.reset(); AC.speed = 200.0; AC.position_units = Vector3(0, 4100, 0)
	AC.step()
	print("CEILING: isStalling=%s (expect true)" % AC.isStalling)

	# 7) High-G turn: accel+brake+hard pull
	AC.reset(); AC.speed = 150.0; AC.position_units = Vector3(0, 500, 0)
	AC.accelValue = 1.0; AC.brakeValue = 1.0; AC.pitchValue = -1.0
	var f0 := 0.0
	for t in range(int(0.5 / AC.FIXED_DT)):
		AC.step()
		if t == 0:
			f0 = AC.rotateValue.x
	print("HIGHG: turning=%s pitch_rate=%.1f deg/s (factor decays 2x->1x of %.1f)" % [AC.isHighGTurning, AC.rotateValue.x, AC.pitchAmount])

	# 8) Autopilot: from hard attitude, both yaws held
	AC.reset(); AC.q = AC.u_euler(-30.0, 45.0, 60.0)  # 30 up, yaw 45, roll right 60
	AC.speed = 150.0; AC.position_units = Vector3(0, 500, 0)
	AC.yawLValue = 1.0; AC.yawRValue = 1.0
	for t in range(int(3.0 / AC.FIXED_DT)):
		AC.step()
	e = AC.unity_euler()
	print("AUTOPILOT: pitch=%.1f roll=%.1f (expect ~0/~0), yaw held=%.0f" % [e.x, e.z, e.y])

	# 9) Gravity: nose down 30 deg, no throttle -> speed must rise
	AC.reset(); AC.q = AC.u_euler(30.0, 0.0, 0.0); AC.speed = 60.0
	AC.position_units = Vector3(0, 500, 0)
	for t in range(int(5.0 / AC.FIXED_DT)):
		AC.step()
	print("GRAVITY_DIVE: speed=%.1f (expect > 60)" % AC.speed)

	quit()
