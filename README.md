# Side B — OperationZERO Physics, ported 1:1 to Godot

A faithful port of `AircraftController.cs` from
[lunetis/OperationZERO](https://github.com/lunetis/OperationZERO) (Unity AC Zero
remake). Every formula, constant, and quirk is preserved — this is the control
group for the measured-AC4 model in `../godot/` (Side A).

## What "faithful" means here
- Same fixed timestep (Unity `fixedDeltaTime` = 0.02 s)
- Same serialized values, taken from the actual ZERO.unity scene file:
  maxSpeed 250, minSpeed 15, defaultSpeed 60, throttleAmount 5, accelAmount 25,
  brakeAmount 100, calibrateAmount 50, rollAmount 125, pitchAmount 50,
  yawAmount 30, rotateLerpAmount 4, highGFactor 1.2, highGTurnTime 2,
  stallSpeed 25, stallHeight 4000, gravityFactor 5
- Same units: speed m/s (HUD shows ×10), altitude world units (HUD ×5)
- Same state machine: throttle lerp, accel/brake easing, calibration pull to
  defaultSpeed, gravity-by-pitch, stall-nose-to-ground, ceiling stall,
  high-G turn (accel+brake+hard pull, factor decays 2x->1x over 2 s budget),
  autopilot level-off (both rudders held) — including the original's
  autopilot y-wrap bug, kept verbatim
- Unity quaternion/Euler semantics (left-handed, YXZ, eulerAngles [0,360))
  reimplemented over Godot quaternions; verified by 500-sample round-trip
  (max error 0.0001 deg)

## Run
    godot --path .                    # interactive
    godot --headless --path . --script parity_test.gd   # parity suite

## Controls (their scheme)
- Arrows      pitch / roll (DOWN = pull up, like Side A)
- 3 / 1       accelerate / brake — springy throttle: target = accel - brake,
              lerps back to 0 (neutral) on release; speed calibrates toward
              defaultSpeed (60 m/s = HUD 600) when lever is centered
- 3 + 1 + hard pull   HIGH-G turn (their original trigger)
- Q / E       rudder left / right
- Q + E       autopilot level-off (wings + pitch, yaw untouched)
- Esc         quit

## Known quirks (faithful, not bugs introduced here)
- Braking below 25 m/s triggers the stall, which forces the nose ~straight
  down; the jet then dives and gravity overshoots the calibration target.
  That is exactly what the Unity original does.
- No bank->turn coupling: turning is rudder-only (yaw input).
- No induced-drag bleed: speed only changes via throttle, calibration, gravity.

## Files
- aircraft_controller.gd  — the port (physics core)
- main.gd / main.tscn     — interactive stage (sky, ground, cones, jet, chase cam, HUD)
- parity_test.gd          — mapping round-trip + behavior scenarios
- obj_loader.gd, f-4e-basic.obj — shared jet mesh from Side A

## License
This project is licensed under the BSD-3-Clause (see `LICENSE`).

## Third-party assets & code
- `aircraft_controller.gd` is a port of `AircraftController.cs` from
  [OperationZERO](https://github.com/lunetis/OperationZERO),
  Copyright (c) 2021 lunetis, MIT license — full text in `LICENSE.OperationZERO.txt`.
- `f-4e-basic.obj` (jet model) from [YSFLIGHT](https://github.com/captainys/YSFLIGHT),
  Copyright 2022 Soji Yamakawa (CaptainYS), BSD-3-Clause license —
  full text in `LICENSE.f-4e-basic.obj`.
