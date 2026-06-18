# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS 2 (Jazzy) workspace for "DW robot," a differential-drive robot. All packages use the
`ament_python` build type. There are four packages under `src/`:

- `dw_robot_base` — the core package: cmd_vel arbitration/safety, motor drivers (serial + Modbus RTU),
  odometry, keyboard/RF teleop, and a USB camera node. Almost all real work happens here.
- `dw_robot_bringup` — top-level launch convenience package (currently just `sim_bringup.launch.py`).
- `dw_robot_description` — URDF/xacro robot model and RViz display launch.
- `hwt901b_driver` — standalone driver node for the WitMotion HWT901B IMU (serial protocol parser).

There is a stray top-level `motordriver.py` (not part of any ROS package) — a one-off script for sending
a raw Modbus packet to change an MDROBOT motor controller's slave ID. It is not imported by anything.

## Build / lint / test

Standard `colcon` workflow from the workspace root (`/home/jh/dw_robot_ws`):

```bash
colcon build --symlink-install      # build all packages
colcon build --symlink-install --packages-select dw_robot_base   # build a single package
source install/setup.bash           # required before running any node/launch file
```

Each package's tests are the ament boilerplate (`test_copyright.py`, `test_flake8.py`, `test_pep257.py`)
run via pytest/colcon:

```bash
colcon test --packages-select dw_robot_base
colcon test-result --verbose
```

To run a single test file directly (faster iteration, no colcon):

```bash
python3 -m pytest src/dw_robot_base/test/test_flake8.py
```

There are no unit tests for node logic itself — only the ament linters (flake8, pep257, copyright header
check). Note `pnt50_modbus.py` is intentionally ROS-free so it can be unit-tested or reused standalone.

## Architecture

### cmd_vel pipeline (topic chain)

Robot motion commands flow through a fixed chain of small, single-purpose nodes connected only by topics
(never by direct calls). Understanding this chain is necessary before touching any one node, since each
node assumes specific upstream/downstream behavior:

```
keyboard_brake_node  ─┐
                       ├─→ cmd_vel_mux_node ─→ cmd_vel_safety_node ─→ {fake_diff_drive_node | base_controller_node}
rf_arduino_node       ─┘   (/cmd_vel_raw)      (/cmd_vel_safe)
(/cmd_vel_keyboard,
 /cmd_vel_rf)
```

- **`cmd_vel_mux_node`**: selects between the keyboard and RF input sources based on the
  `input_source` parameter (runtime-changeable via `ros2 param set`), zeroing output if the selected
  source has timed out or is invalid.
- **`cmd_vel_safety_node`**: clamps velocity/acceleration to configured limits and zeroes commands on
  timeout. This is the single place velocity/acceleration limits are enforced — node-specific
  per-wheel clamping downstream is just normalization, not a safety limit.
- **`base_controller_node`**: converts `Twist` → differential-drive wheel velocities, normalizes them
  to `[-1, 1]`, publishes `/wheel_cmd` (`[left_vel, right_vel, left_norm, right_norm]`), and optionally
  also writes simple ASCII serial commands (`M,<l>,<r>`, `S`, `B,1`/`B,0`) directly to a board when
  `output_mode: serial`. It implements its own brake state machine (`use_brake_protocol`,
  `brake_on_timeout`, `brake_on_zero_cmd`) independent of `pnt50_driver_node`'s.
- **`fake_diff_drive_node`**: a simulation stand-in for the real base — integrates `/cmd_vel_safe`
  directly into `/odom`, `/tf`, and `/joint_states` (no `/wheel_cmd` involved). Used in `*_fake*`
  launch files in place of the real motor stack.
- **`pnt50_driver_node`**: consumes `/wheel_cmd`, converts normalized wheel speeds to RPM, and talks
  Modbus RTU (function codes 0x06/0x10, CRC16) to dual MDROBOT/PNT50 motor controllers over serial.
  Publishes `/pnt50/target_rpm` and `/pnt50/comm_ok` for monitoring. Has its own brake handling via
  PID 175 (electric brake), separate from `base_controller_node`'s brake protocol — the two are not
  meant to run in the same launch file pair (see launch files below).
- **`odom_from_pnt50_feedback_node`**: separate from the cmd_vel chain — integrates wheel tick feedback
  (`/pnt50_feedback`, `[left_ticks, right_ticks]`) into `/odom` + `/tf` using basic differential-drive
  dead reckoning.

Brake is a parallel side channel (`/brake_cmd`, `std_msgs/Bool`), published by `keyboard_brake_node` or
`rf_arduino_node` and consumed independently by `base_controller_node` and `pnt50_driver_node`.

### Modbus / motor control

`pnt50_modbus.py` is a standalone, ROS-independent Modbus RTU client (`Pnt50ModbusClient`) for
MDROBOT/PNT50 controllers, with PID constants, CRC16, and read/write-word helpers. `pnt50_driver_node.py`
re-implements equivalent low-level framing/CRC logic inline rather than using this client — the two are
not currently unified, so a protocol fix may need to be applied in both places.

### Input sources

- `rf_arduino_node` reads a 9-field CSV line from an Arduino over serial
  (`throttle,steering,enable,brake,signal_ok,pulse1,pulse2,rpm1,rpm2`), converting RC PWM pulses to a
  `Twist` plus brake state and motor feedback. It accepts older 4- and 5-field formats for backward
  compatibility with earlier Arduino firmware.
- `keyboard_brake_node` is a terminal-raw-mode teleop (`i,j,k,l,u,o,m,.` for motion, `k`=brake,
  `space`=stop without brake, `q/z/w/x` to adjust speed step).
- `rc_pnt50_serial_node` is a simpler/older variant that just republishes the raw Arduino CSV as a
  `Float32MultiArray` (not wired into the cmd_vel chain; appears to predate `rf_arduino_node`).

### Launch files (`dw_robot_base/launch/`)

Pick the launch file matching the actual hardware combination in use — each wires together a specific
subset of nodes and is not just a superset/subset of the others:

- `keyboard_fake.launch.py` / `rf_fake.launch.py`: teleop → mux → safety → **fake** diff drive (simulation,
  no real motors), with RViz.
- `keyboard_real.launch.py` / `keyboard_pnt50.launch.py` / `rf_pnt50.launch.py`: teleop → mux → safety →
  `base_controller_node` (+ `pnt50_driver_node` in the `_pnt50` variants) against **real** hardware.
- `rf_pnt50_rviz.launch.py`: a separate, narrower path — robot_state_publisher +
  `odom_from_pnt50_feedback_node` + RViz only, for visualizing odometry from real wheel feedback without
  the full teleop/safety/control chain.
- `pnt50_driver.launch.py`: just the PNT50 motor driver in isolation.

All launch files load YAML parameter files from `dw_robot_base/config/*.yaml` via
`FindPackageShare`/`get_package_share_directory`, so parameter changes belong in the YAML files, not as
launch-file literals (with the exception of a few inline overrides, e.g. `input_source` and the camera
params in `rf_pnt50.launch.py`).

### Topic timeouts everywhere

Nearly every node in the cmd_vel/teleop chain independently tracks "time since last message" and zeroes
its output (and/or requests brake) on timeout — `cmd_vel_mux_node`, `cmd_vel_safety_node`,
`base_controller_node`, `pnt50_driver_node`, and `rf_arduino_node` each have their own
`*_timeout`/`cmd_timeout` parameter. When debugging "robot doesn't move" or "robot won't stop," check
which of these independent timeouts is firing — they are not centralized.

### HWT901B IMU driver

`hwt901b_driver/hwt901b_node.py` runs its own serial-reading thread (`serial_read_loop`) decoupled from
the ROS publish timer (`publish_data`), parsing the WitMotion binary frame protocol (0x55 header,
packet types 0x51/0x52/0x53/0x54 for accel/gyro/angle/mag, checksum = sum of first 10 bytes). It is
standalone — not currently referenced by any launch file in this workspace.
