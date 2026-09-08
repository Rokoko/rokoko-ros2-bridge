# rkk-ros2-bridge

A ROS2 bridge for Rokoko SmartGlove solved-hand data: connects to
`rkk-hand-solver`'s solved-hand output (RGMP v2) and republishes it as
ROS2 topics, an opt-in TF broadcast, and a yaw-calibration service.

Runs natively against the system ROS2 install — no separate Python
environment/dependency manager. Minimal dependencies by design.

> **Note:** ROS2's Python bindings (`rclpy`, message
> packages, etc.) aren't distributed via PyPI — they only exist as part of a
> ROS2 distro install, made available by sourcing `/opt/ros/<distro>/setup.bash`
> into the system Python. A `uv`/`venv` environment can't see those, so this
> package is meant to run against the system interpreter, not an isolated
> virtual environment. If isolation is ever needed (a clean environment, a
> different OS/distro version, CI), the right tool for that is a **Docker
> container** with ROS2 installed inside it — not a Python venv.

## Prerequisites

- **Ubuntu 26.04** (or whatever your ROS2 distro targets)
- **[ROS2 Lyrical Luth](https://docs.ros.org/en/lyrical/Get-Started/Installation/Ubuntu-Install-Debs.html)**
  — follow the official Debian install guide. Heads up: `ros-lyrical-desktop`
  is a large install (hundreds of packages, a couple GB) — expect it to take
  a few minutes.
- **colcon** — not bundled with `ros-lyrical-desktop`, install separately:
  ```sh
  sudo apt install python3-colcon-common-extensions
  ```
- **[Rokoko Device SDK](https://sdk.rokoko.com/)** — provides `rokoko-sdk`
  (the SmartGlove driver) and `rkk-hand-solver` (the hand solver), both of
  which this bridge connects to. See
  [Running the upstream chain](#running-the-upstream-chain) below.

## Repo layout

```
ros2_ws/
  src/
    rkk_hand_msgs/       # interfaces: HandDescription.msg, HandJoints.msg
    rkk_hand_bridge/      # the node: connects, decodes, publishes
    rkk_hand_bringup/     # launch files that bring up the whole chain
```

`ros2_ws` is the colcon workspace root — packages live under `ros2_ws/src`,
and `colcon build` is run from there.

## Building

```sh
source /opt/ros/lyrical/setup.bash   # or add this to ~/.bashrc
cd ros2_ws
colcon build
source install/setup.bash            # needed in every new shell you run/launch from
```

## Running

First, bring up the upstream chain (the driver + solver this bridge
connects to) — see [Running the upstream chain](#running-the-upstream-chain)
below for what those two processes are and why nothing else is needed.

**Option A — one command starts everything**, including the solver:

```sh
ros2 launch rkk_hand_bringup smartglove_hands.launch.py
```

This spawns the solver and the bridge together, using the command in
`config/upstream.yaml` — `rkk-hand-solver --emit-openxr` plus its
`--driver-arg` values, which makes the solver start `rokoko-sdk` itself.
See [Configuring the solver and driver](#configuring-the-solver-and-driver)
to change that command.

**Option B — attach to a solver that's already running** (e.g. you started
it by hand, or another tool already has it up):

```sh
ros2 launch rkk_hand_bringup smartglove_hands.launch.py spawn_solver:=false
```

**Option C — just the bridge**, no bringup package involved:

```sh
ros2 launch rkk_hand_bridge hand_bridge.launch.py
```

Useful launch arguments (pass as `name:=value`): `solver_host`,
`solver_port` (default `12277`), `publish_tf` (default `false`),
`parent_frame_id` (required if `publish_tf:=true`), `stamp_source`
(default `auto`), `calibrate_facing_rad`. `rkk_hand_bringup` additionally
takes `spawn_solver` (default `true`) and `config` — see below.

### Configuring the solver and driver

The solver and driver are plain subprocesses, not ROS nodes, so their
arguments don't come from a ROS parameter file. `rkk_hand_bringup` reads
them from `config/upstream.yaml`, which *is* the default — there is no
second set of defaults hidden in the launch file:

```yaml
solver:
  executable: rkk-hand-solver
  args: ["--emit-openxr"]
  driver_args: ["--auto-stream-usb", "-ef", "1024"]
```

`args` is passed through verbatim, and each entry of `driver_args` is
forwarded as `--driver-arg=<value>`. `config:=/path/to/my.yaml` merges
over this file, so a partial file only needs the keys it changes.

Whether a driver gets started is therefore decided by what you put in
`args`, not by the launch file. With the file above the solver spawns
`rokoko-sdk` itself, applying its own built-in defaults
(`-vv --auto-stream-usb`) with `driver_args` appended after them — which
means `driver_args` can only *add* arguments, never remove a solver
default. Put `--no-driver` in `args` to attach to a driver you started
yourself.

To remove a solver default you need the launch file to run the driver, so
add an optional `driver` section giving the complete command. That is the
case for **gloves on WiFi rather than USB**: WiFi is the driver's native
path (it is a UDP server on `--driver-udp-port`, default `14041`), and
`--auto-stream-usb` exists only to make USB/serial devices behave the way
WiFi ones already do.

```yaml
solver:
  args: ["--emit-openxr", "--no-driver"]
driver:
  executable: rokoko-sdk
  args: ["-vv", "-ef", "1024"]
```

A `driver` section without `--no-driver` in `solver.args` is rejected at
launch, since both would start a driver. WiFi gloves must also be on the
same network and pointed at this machine's LAN address, not `127.0.0.1`.

`spawn_solver:=false` starts the bridge alone.

## Verifying it's working

With the bridge running (any option above), in another sourced shell:

```sh
ros2 topic list
ros2 topic echo /rkk_hand_bridge/hand/description --once
ros2 topic echo /rkk_hand_bridge/hand/joints --once
ros2 topic echo /diagnostics --once
```

To calibrate yaw (hold a hand toward whatever direction `calibrate_facing_rad`
represents, default straight ahead, then call):

```sh
ros2 service call /rkk_hand_bridge/calibrate std_srvs/srv/Trigger {}
```

`/diagnostics` should show `WARN`/"connected, yaw uncalibrated" before this
and `OK`/"connected" immediately after a successful call.

## Visualizing in RViz

`HandJoints` is a custom message, so RViz has no built-in display for it —
the only thing RViz can draw directly is the opt-in **TF** broadcast. Launch
with `publish_tf` on and a real `parent_frame_id` (TF needs a non-empty
parent; the bridge refuses to guess one):

```sh
ros2 launch rkk_hand_bringup smartglove_hands.launch.py publish_tf:=true parent_frame_id:=world
```

Then, in another sourced shell:

```sh
rviz2
```

In RViz:

1. Set **Fixed Frame** (top-left, Global Options) to `world` (or whatever
   you passed as `parent_frame_id`).
2. **Add** → **By display type** → **TF**.

That's it — every joint of every connected hand shows up automatically, no
per-joint configuration needed (all 26 joints × however many hands are
connected get broadcast every frame). Two things worth doing to make it
readable rather than just correct:

- The TF display's **Marker Scale** defaults to a size meant for room-scale
  robots; a hand's joints are centimeters apart, so turn it down (try
  `0.02`–`0.05`) or the axis markers will overlap into a blob.
- Enable **Show Names** on the TF display if you want to tell joints apart
  by label (`rkk_<hand>_hand_xr_<joint_name>`) rather than by position alone.

Until `~/calibrate` has been called, expect the hand to be rotated by some
arbitrary amount around the vertical axis — that's expected (see the design
docs' timestamp/calibration notes), not a bug.

## Running the upstream chain

This bridge is a plain RGMP v2 client of `rkk-hand-solver`'s solved-hand
output — it needs exactly two other processes running, nothing more:

```sh
# 1. The SmartGlove driver. -ef 1024 enables experimental epoch timestamps,
#    which this bridge prefers when available (see the timestamp handling
#    in the design docs) — verify the exact flag name against
#    `rokoko-sdk --help` on your build, long and short forms can vary.
rokoko-sdk -vv --auto-stream-usb -ef 1024

# 2. The hand solver, attaching to the driver already running above.
#    --emit-openxr is the only group this bridge reads.
rkk-hand-solver --emit-openxr --no-driver
```

`rkk_hand_bringup`'s launch file (Option A/B above) manages step 2 for you;
step 1 (the driver) is always your responsibility to start separately.

## Status

Implemented and verified against real, physically-connected SmartGlove
hardware: `rkk_hand_msgs` interfaces, the full `rkk_hand_bridge` node
(connect/decode/reconnect, `hand/description`, `hand/joints`, opt-in TF,
`~/calibrate`, `/diagnostics`), and both launch packages.

A few things are known-incomplete rather than silently assumed solid —
see `.claude/plan.md`'s "Bugs found during implementation" and
"Known-untested paths" sections for the current list (not tracked in git,
so ask if you want a copy).
