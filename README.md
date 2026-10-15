# Nested PID Hover-Transition Dashboard

This branch preserves the earlier `v1` implementation. Its main function is an interactive nested altitude/vertical-velocity PID dashboard with manual throttle and a transition into hover mode. It overlaps substantially with the `version/throttle-hover` branch, so it is retained as a comparison point rather than treated as a distinct control architecture.

## Functionality

- Offers a manual throttle input.
- Uses an altitude loop to derive a vertical-velocity target.
- Uses a vertical-velocity loop to adjust thrust.
- Includes hover-transition logic to hold the altitude captured when hover mode is enabled.
- Provides a dashboard-oriented simulation workflow and visualization/telemetry as implemented in the source.

## What makes this version useful

Use it to compare the earlier `v1` implementation with the more clearly named `version/throttle-hover` version. Differences should be judged by inspecting and running the MATLAB source; the branch name alone does not imply better performance or stability.

## Run

From the repository root in MATLAB:

```matlab
run("src/nested_pid_hover_transition.m")
```

## Requirements

- MATLAB with the UI components used by the script.
- Robotics System Toolbox may be required for URDF import and visualization functions such as `importrobot`, `homeConfiguration`, or `show`.
- The uploaded workspace did not include `drone.urdf`. Supply a compatible model if required by the source.

## Repository contents

```text
README.md
src/
  nested_pid_hover_transition.m
```

The branch intentionally contains MATLAB `.m` source only; MATLAB Live Scripts (`.mlx`) are not included.

## Limitations and validation status

This is a simplified educational simulation, not a flight-ready controller. It does not model full 6-DOF quadrotor dynamics. The code has not been executed or validated in MATLAB as part of this repository packaging. Verify behavior, controller tuning, and mode transitions in MATLAB before relying on the results.

## License

No license is provided. Add a license before public release if you want to grant reuse, modification, or redistribution rights.
