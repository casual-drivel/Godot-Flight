extends Node3D
# Interactive scene: OperationZERO physics, Godot stage.
# Keys: arrows pitch/roll (DOWN=pull up), 1=brake 3=accel (both+pull = HIGH-G),
# Q/E = rudder, Q+E held = autopilot.

var ac
var jet: MeshInstance3D
var cam: Camera3D
var hud: Label
var _acc := 0.0
var _jet_pos := Vector3(0.0, 400.0, 0.0)

func _ready() -> void:
	ac = preload("res://aircraft_controller.gd").new()
	ac.reset()
	ac.position_units = _jet_pos

	# jet mesh
	jet = MeshInstance3D.new()
	var jetmat := StandardMaterial3D.new()
	jetmat.albedo_color = Color(0.75, 0.78, 0.82)
	jetmat.cull_mode = BaseMaterial3D.CULL_DISABLED
	jet.mesh = preload("res://obj_loader.gd").load_obj("res://f-4e-basic.obj")
	jet.material_override = jetmat
	add_child(jet)

	# sky + fog
	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	var skymat := ProceduralSkyMaterial.new()
	skymat.sky_top_color = Color(0.16, 0.42, 0.82)
	skymat.sky_horizon_color = Color(0.62, 0.75, 0.92)
	skymat.ground_bottom_color = Color(0.25, 0.32, 0.38)
	skymat.ground_horizon_color = Color(0.62, 0.75, 0.92)
	sky.sky_material = skymat
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.fog_enabled = true
	env.fog_light_color = Color(0.75, 0.83, 0.94)
	env.fog_density = 0.00002
	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)

	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-55.0, 30.0, 0.0)
	sun.light_energy = 1.2
	sun.shadow_enabled = true
	add_child(sun)

	# ground
	var ground := MeshInstance3D.new()
	var gm := PlaneMesh.new()
	gm.size = Vector2(40000, 40000)
	ground.mesh = gm
	var gmat := StandardMaterial3D.new()
	gmat.albedo_color = Color(0.30, 0.42, 0.25)
	ground.material_override = gmat
	add_child(ground)

	# scattered mountains so speed reads visually
	var rng := RandomNumberGenerator.new()
	rng.seed = 7
	var mmat := StandardMaterial3D.new()
	mmat.albedo_color = Color(0.35, 0.38, 0.34)
	for i in 60:
		var m := MeshInstance3D.new()
		var cm := CylinderMesh.new()
		cm.top_radius = rng.randf_range(2.0, 30.0)
		cm.bottom_radius = rng.randf_range(150.0, 400.0)
		cm.height = rng.randf_range(200.0, 900.0)
		m.mesh = cm
		m.material_override = mmat
		m.position = Vector3(rng.randf_range(-9000, 9000), cm.height * 0.5 - 5.0,
			rng.randf_range(-9000, 9000))
		add_child(m)

	cam = Camera3D.new()
	add_child(cam)
	cam.current = true
	cam.far = 30000.0

	hud = Label.new()
	hud.position = Vector2(16, 12)
	hud.add_theme_font_size_override("font_size", 20)
	add_child(hud)

func _feed_inputs() -> void:
	ac.pitch_value = (1.0 if Input.is_key_pressed(KEY_UP) else 0.0) \
		- (1.0 if Input.is_key_pressed(KEY_DOWN) else 0.0)
	ac.roll_value = (1.0 if Input.is_key_pressed(KEY_LEFT) else 0.0) \
		- (1.0 if Input.is_key_pressed(KEY_RIGHT) else 0.0)
	ac.accel_value = 1.0 if Input.is_key_pressed(KEY_3) else 0.0
	ac.brake_value = 1.0 if Input.is_key_pressed(KEY_1) else 0.0
	ac.yaw_l_value = 1.0 if Input.is_key_pressed(KEY_Q) else 0.0
	ac.yaw_r_value = 1.0 if Input.is_key_pressed(KEY_E) else 0.0

func _physics_process(delta: float) -> void:
	_acc += delta
	while _acc >= ac.FIXED_DT:
		_feed_inputs()
		ac.step()
		_acc -= ac.FIXED_DT
	# render-interpolate between the last two 50 Hz states so the 60 Hz
	# display sees smooth motion instead of 1-1-2-step judder
	var alpha: float = clampf(_acc / ac.FIXED_DT, 0.0, 1.0)
	_jet_pos = ac.prev_position_units.lerp(ac.position_units, alpha)
	var q_render: Quaternion = ac.prev_attitude.slerp(ac.attitude, alpha)
	jet.global_basis = Basis(q_render)
	jet.global_position = _jet_pos

	# chase cam locked to attitude
	var b := Basis(q_render)
	var cam_target := _jet_pos + b * Vector3(0.0, 3.0, 20.0)
	cam.global_position = cam.global_position.lerp(cam_target, 1.0 - exp(-8.0 * delta))
	cam.global_basis = cam.global_basis.slerp(b, 1.0 - exp(-8.0 * delta))

	var flags := ""
	if ac.is_stalling:
		flags += "  STALL"
	if ac.is_high_g_turning:
		flags += "  HIGH-G"
	if ac.is_autopilot:
		flags += "  AP"
	hud.text = "SPD %4d   ALT %5d   HDG %03d\nTHR %+3.0f%%   [OperationZERO-Godot]%s" % [
		ac.display_speed(), ac.display_altitude(), int(ac.display_heading()),
		ac.throttle * 100.0, flags]

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and event.keycode == KEY_ESCAPE:
		get_tree().quit()
