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
`publish_markers` (default `false`), `marker_lifetime_s` (default
`0.5`), `marker_rate_hz` (default `30`, shared across hands),
`marker_hand_spacing_m` (default `0.45`), `marker_joint_scale`,
`marker_max_joint_radius_m`, `parent_frame_id` (required if `publish_tf` or `publish_markers`
is set), `stamp_source` (default `auto`), `calibrate_facing_rad`,
`calibrate_max_disagreement_rad` (default 30 degrees in radians).
`rkk_hand_bringup` additionally
takes `spawn_solver` (default `true`), `config` and `rviz` (default
`false`) — see below.

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

**The offset is shared by every connected hand.** It corrects for
magnetic-north referencing, which is a property of the room rather than
of a glove, so one call calibrates all of them. The measurement comes
from a single hand — the lowest `device_id`, chosen so repeated calls
give the same answer — and the reply names it:

```
calibrated yaw offset to -2.834 rad from the left hand (device 843143769)
```

Because one hand's heading is applied to all, the hands have to agree.
If they are pointing more than `calibrate_max_disagreement_rad` apart
(default 30 degrees) the call is refused rather than silently applying
one hand's correction to the other:

```
hands disagree by 45 degrees; calibrate with every hand facing the same
way, or raise calibrate_max_disagreement_rad (now 30 degrees)
```

Nothing changes when a call is refused — the previous offset stays in
place, and `/diagnostics` keeps reporting uncalibrated if it was.

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

```sh
ros2 launch rkk_hand_bringup smartglove_hands.launch.py \
  publish_markers:=true publish_tf:=true parent_frame_id:=world rviz:=true
```

`rviz:=true` just starts `rviz2` alongside the bridge; no config is
passed, so RViz opens with its own settings and keeps whatever you last
saved. Leave it off and run `rviz2` yourself if you prefer.

**Setting the display up, once.** In RViz:

1. **Global Options** (top-left) → **Fixed Frame** → `world`, or whatever
   you passed as `parent_frame_id`.
2. **Add** → **By topic** → `/rkk_hand_bridge/hand/markers` →
   **MarkerArray**.

Then **File → Save Config** (Ctrl-S) and RViz will reopen like that every
time.

RViz needs the fixed frame to exist in TF before it will draw anything
positioned in it, which is why the command above also passes
`publish_tf:=true` — it costs you no clutter unless you add a TF display,
it just gives the frame something to exist in. That does tie the frame's
existence to the data stream, so if the gloves drop out the frame goes
with them. Anchoring it independently avoids that:

```sh
ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id rkk_hand_anchor
```

**If the hand flickers**, check any **Grid** display you have added. A
fine grid (10cm cells, say) puts semi-transparent lines straight through
the markers at z=0, and transparent geometry intersecting the spheres
makes the renderer's depth sorting unstable — which looks like the whole
hand blinking. A coarse grid (1m cells, RViz's default) keeps its lines
clear of the hand entirely.

Every connected hand appears automatically, left and right in different
colors, under a `rkk_<hand>_hand/joints` and `rkk_<hand>_hand/bones`
namespace each — so you can toggle spheres and skeleton independently in
the display's **Namespaces** list.

Markers are drawn fully opaque. Anything translucent lands in the
renderer's transparent queue, where 26 overlapping spheres cost fill rate
proportional to the window's pixel count — which showed up as the hand
flickering once the RViz window was maximised, and did not improve with a
lower `marker_rate_hz`, since the cost is per drawn frame rather than per
message.

Connected hands are drawn side by side rather than on top of each other:
both gloves report wrist-relative poses, so at true scale they sit in the
same place and overlap into one tangle. `marker_hand_spacing_m` (default
`0.45`) fans them out along +Y, centred on `parent_frame_id`'s origin —
two hands land at ∓0.225m, three at −0.45/0/+0.45, and a single hand
stays exactly at the origin. `marker_hand_spacing_m:=0.0` stacks them
again.

Slots are assigned by sorted `device_id`, so a hand keeps its side of the
scene across reconnects instead of hopping when packets arrive in a
different order.

**This shifts the drawing only.** `hand/joints` and `/tf` publish the
true, unshifted poses — anything consuming the data still sees both hands
where they physically are. Only what RViz draws is spread out.

Marker namespaces and TF frame ids are keyed on handedness, so **two
gloves of the same handedness are not supported** — one left and one
right is the case this handles. `hand/joints` still carries `device_id`
and stays unambiguous either way.

Markers carry a `marker_lifetime_s` (default `0.5`) lifetime, so if the
stream stops they fade out instead of leaving a frozen hand on screen; a
hand that disconnects cleanly clears itself immediately.

They are also throttled to `marker_rate_hz` (default `30`), well below
the solver's frame rate. A hand is 27 markers per frame, so at the
solver's 83Hz that is ~2250 markers/second, and markers are for a human
watching a screen rather than for consumers of the data. The throttle
brings it to ~750/s.

**`marker_rate_hz` is a total budget, not a per-hand rate.** Every
connected hand shares it, so two gloves get roughly 15Hz each rather than
30Hz each, and what RViz has to draw stays flat as gloves are added
instead of doubling. Measured with two hands fed at 83Hz each: ~28 marker
arrays/second in total, the same as one hand.

This affects the drawing only. `hand/joints` and `/tf` carry every frame
of every hand regardless — with two hands the same run measured `/tf` at
166Hz, both hands at full rate. Turn markers down, or off, without
touching what your consumers receive.

`marker_rate_hz:=0.0` publishes one array per frame per hand. Note it is
a double, so `0.0` — a bare `0` is rejected as the wrong parameter type.

To see joint frames as well, add a **TF** display yourself (**Add** →
**By display type** → **TF**) and turn its **Marker Scale** down to
`0.02`–`0.05` — TF's default is meant for room-scale robots and a hand's
joints are centimeters apart. **Show Names** labels them
`rkk_<hand>_hand_xr_<joint_name>`.

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
