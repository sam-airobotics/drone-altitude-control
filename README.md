# Motor-Lag Hover Dashboard

An interactive MATLAB prototype for studying vertical drone motion, throttle response, motor spool-up lag, and hover control. This is the recommended starting point among the supplied versions because it combines interactive controls, live telemetry, and a fallback visualization when URDF import fails.

## Functionality

- Provides a throttle slider for manual input.
- Includes a hover-mode toggle that captures the current altitude as the target.
- Uses altitude and vertical-velocity feedback to adjust thrust.
- Models motor response with a first-order lag so RPM does not change instantaneously.
- Displays telemetry such as altitude, vertical velocity, thrust, and RPM.
- Attempts to load `drone.urdf` for visualization and falls back to a simple marker if import fails.

## Model overview

The simulation approximates vertical motion using \(\ddot z = (T-mg)/m\), with thrust approximated from motor RPM squared. These are simplified assumptions for learning and are not calibrated to a real motor/propeller system. The model does not simulate full 6-DOF quadrotor dynamics, attitude, horizontal movement, aerodynamic effects, or battery behavior.

## Run

From the repository root in MATLAB:

```matlab
run("src/motor_lag_hover_dashboard.m")
```

## Requirements

- MATLAB with the UI components used by the script.
- Robotics System Toolbox is recommended for `importrobot`, `homeConfiguration`, and `show` visualization functions.
- The original workspace did not include `drone.urdf`. If you have a compatible model, put it in the repository root or update the path used by the script.

## Repository contents

```text
README.md
src/
  motor_lag_hover_dashboard.m
```

The branch intentionally contains MATLAB `.m` source only; MATLAB Live Scripts (`.mlx`) are not included.

## Limitations and validation status

This is an educational prototype, not a flight-ready controller. The code has not been executed or validated in MATLAB as part of this repository packaging. Check controller stability, units, saturation behavior, and model assumptions before relying on the results. Hover mode captures the altitude at activation; enabling it at ground level targets zero altitude.

## License

No license is provided. Add a license before public release if you want to grant reuse, modification, or redistribution rights.
