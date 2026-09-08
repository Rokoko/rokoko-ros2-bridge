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

This starts the driver, the solver and the bridge together, using the
commands in `config/upstream.yaml`. See
[Configuring the solver and driver](#configuring-the-solver-and-driver)
to change them.

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
`publish_markers` (default `false`), `marker_lifetime_s` (default
`0.5`), `marker_rate_hz` (default `30`), `marker_joint_scale`,
`marker_max_joint_radius_m`, `parent_frame_id` (required if `publish_tf` or `publish_markers`
is set), `stamp_source` (default `auto`), `calibrate_facing_rad`. `rkk_hand_bringup` additionally
takes `spawn_solver` (default `true`), `config`, `rviz` (default
`false`) and `rviz_config` — see below.

### Configuring the solver and driver

The solver and driver are plain subprocesses, not ROS nodes, so their
arguments don't come from a ROS parameter file. `rkk_hand_bringup` reads
them from `config/upstream.yaml`, which *is* the default — there is no
second set of defaults hidden in the launch file:

```yaml
solver:
  executable: rkk-hand-solver
  args: ["--emit-openxr", "--no-driver"]

driver:
  executable: rokoko-sdk
  args: ["-vv", "--auto-stream-usb", "-ef", "1024"]
```

Both `args` lists are passed through verbatim, so what you read above is
exactly what runs. `config:=/path/to/my.yaml` merges over this file, so a
partial file only needs the keys it changes.

**Why the driver is started here rather than by the solver.**
`rkk-hand-solver` will happily spawn `rokoko-sdk` itself — that is its
default — but when it does, it prepends its own `-vv --auto-stream-usb`
and `--driver-arg` can only *append* after those ("Extra argument for the
driver, after the defaults"). So a driver argument set that way appears
in addition to the solver's, never instead of it, and repeating one gives
you `--auto-stream-usb --auto-stream-usb`. Starting the driver from the
launch file and passing the solver `--no-driver` makes this file the only
source of the driver's command line.

The `driver` section is optional. Drop it and put the driver's arguments
in `solver.driver_args` instead — each becomes `--driver-arg=<value>` —
if you would rather the solver managed the driver:

```yaml
solver:
  executable: rkk-hand-solver
  args: ["--emit-openxr"]
  driver_args: ["-ef", "1024"]      # on top of -vv --auto-stream-usb
```

A `driver` section without `--no-driver` in `solver.args` is rejected at
launch, since both would start a driver. To attach to a driver you
started by hand, keep `--no-driver` and delete the `driver` section.

**Gloves on WiFi rather than USB.** WiFi is the driver's native path (it
is a UDP server on `--driver-udp-port`, default `14041`), and
`--auto-stream-usb` exists only to make USB/serial devices behave the way
WiFi ones already do. Harmless with no USB device attached, but to drop
it, delete it from `driver.args`:

```yaml
driver:
  executable: rokoko-sdk
  args: ["-vv", "-ef", "1024"]
```

WiFi gloves must also be on the same network and pointed at this
machine's LAN address, not `127.0.0.1`.

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

`HandJoints` is a custom message, so RViz has no built-in display for it.
The bridge offers two opt-in ways to draw the hand instead, both built from
the same rebased poses it publishes, and both needing a non-empty
`parent_frame_id` (the bridge refuses to guess one):

- **`publish_markers`** — a `visualization_msgs/MarkerArray` on
  `~/hand/markers`: a sphere per joint, sized from the description's
  `joint_radii`, joined by a line skeleton. This is the one that looks
  like a hand.
- **`publish_tf`** — the raw TF broadcast: a full coordinate frame per
  joint. Correct and useful for debugging orientations, but 26 axis
  triads per hand read as clutter rather than as a hand.

`rviz:=true` opens RViz with a packaged config
(`rkk_hand_bringup/rviz/smartglove_hands.rviz`) that already has the
markers display, a 10cm grid and a hand-scale camera set up, so there is
nothing to add by hand:

```sh
ros2 launch rkk_hand_bringup smartglove_hands.launch.py \
  publish_markers:=true publish_tf:=true parent_frame_id:=world rviz:=true
```

`rviz_config:=/path/to/my.rviz` uses your own file instead. To run RViz
separately, point it at the same config:

```sh
rviz2 -d "$(ros2 pkg prefix rkk_hand_bringup)/share/rkk_hand_bringup/rviz/smartglove_hands.rviz"
```

The config's **Fixed Frame** is `world`; if you use a different
`parent_frame_id`, change it to match (Global Options, top-left). RViz
needs that frame to exist in TF before it will draw anything positioned
in it, which is why the command above also passes `publish_tf:=true` —
the config's TF display is off, so this costs you no clutter, it just
gives the frame something to exist in. If you already publish `world`
from elsewhere, markers alone are enough.

Every connected hand appears automatically, left and right in different
colors, under a `rkk_<hand>_hand/joints` and `rkk_<hand>_hand/bones`
namespace each — so you can toggle spheres and skeleton independently in
the display's **Namespaces** list.

Markers carry a `marker_lifetime_s` (default `0.5`) lifetime, so if the
stream stops they fade out instead of leaving a frozen hand on screen; a
hand that disconnects cleanly clears itself immediately.

They are also throttled to `marker_rate_hz` (default `30`), well below
the solver's frame rate. A hand is 27 markers per frame, so at the
solver's 83Hz that is ~2250 markers/second — enough to bury RViz's
renderer and show up as flicker rather than as smoothness. The throttle
brings it to ~750/s. Markers are for a human watching a screen; the data
topics (`hand/joints`, `/tf`) still run at the full frame rate and are
unaffected. `marker_rate_hz:=0` disables the throttle.

To see joint frames as well, tick the config's **Joint frames** (TF)
display on — it is shipped disabled, with **Marker Scale** already
lowered to `0.03` and **Show Names** on, since TF's default scale is
meant for room-scale robots and a hand's joints are centimeters apart.

### Sizing the joint spheres

OpenXR reports the wrist and palm at their true anatomical radii, several
times a fingertip's. Drawn to scale they swamp the fingers — a wrist
sphere comes out around 64mm across on a hand about 180mm long — so
`marker_max_joint_radius_m` (default `0.010`) caps how big a sphere gets
drawn. Capping changes drawn size only: a sphere's *centre* is the exact
joint position either way, so nothing about positional accuracy is lost.
`marker_joint_scale` (default `1.0`) shrinks every sphere proportionally
if you want the whole hand daintier, and `marker_max_joint_radius_m:=0.0`
disables the cap for true-to-life radii.

Both are ordinary node parameters read per frame, so you can dial them in
against a live hand without restarting anything:

```sh
ros2 param set /rkk_hand_bridge marker_max_joint_radius_m 0.008
```

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

Those are the two commands `config/upstream.yaml` runs for you under
Option A; run them by hand instead and use Option B, or `spawn_solver:=false`,
to attach the bridge to what is already up.

## Status

Implemented and verified against real, physically-connected SmartGlove
hardware: `rkk_hand_msgs` interfaces, the full `rkk_hand_bridge` node
(connect/decode/reconnect, `hand/description`, `hand/joints`, opt-in TF,
`~/calibrate`, `/diagnostics`), and both launch packages.

A few things are known-incomplete rather than silently assumed solid —
see `.claude/plan.md`'s "Bugs found during implementation" and
"Known-untested paths" sections for the current list (not tracked in git,
so ask if you want a copy).
