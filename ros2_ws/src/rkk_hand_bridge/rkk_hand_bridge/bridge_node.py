"""rclpy node: republishes rkk-hand-solver's solved-hand stream as ROS2
topics. Thin glue over hand_stream, frame_convert, and timestamps -
those hold the actual logic."""

import math
import threading

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rkk_hand_msgs.msg import HandDescription, HandJoints
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray

from rkk_hand_bridge import constants
from rkk_hand_bridge import frame_convert as fc
from rkk_hand_bridge import rgmp
from rkk_hand_bridge import hand_markers
from rkk_hand_bridge.hand_stream import HandStream
from rkk_hand_bridge.timestamps import StampSource

_HAND_CODE = {"left": HandDescription.HAND_LEFT, "right": HandDescription.HAND_RIGHT}

# Data-driven publishing alone would go quiet exactly when something is
# wrong, and a late subscriber would never learn why.
_DIAGNOSTICS_PERIOD_S = 1.0

# The solver emits joints_local unless asked for the OpenXR group, and
# that is nearly always why a hand arrives without it.
_UNSUPPORTED_HELP = {
    rgmp.MISSING_OPENXR_GROUP: (
        "no joints_openxr group; start the solver with --emit-openxr"
    ),
}


def _unsupported_help(reason: str) -> str:
    if reason.startswith(rgmp.WRONG_JOINT_COUNT):
        _, _, count = reason.partition(":")
        return (
            f"joints_openxr carries {count} joints, not {rgmp.JOINT_COUNT}; "
            "this bridge's messages are fixed at that size"
        )
    return _UNSUPPORTED_HELP.get(reason, reason)

_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
_SENSOR_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)
# RViz's MarkerArray display subscribes reliably, so markers cannot use
# the best-effort profile above. depth=1 instead: a display wants the
# newest hand, and queueing ten stale ones behind a busy renderer is what
# makes it stutter.
_MARKER_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def _hand_code(hand: str) -> int:
    return _HAND_CODE.get(hand, HandDescription.HAND_LEFT)


def _stamp_from_ns(stamp_ns: int) -> TimeMsg:
    return TimeMsg(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)


def _to_pose(position, orientation) -> Pose:
    return Pose(
        position=Point(x=position[0], y=position[1], z=position[2]),
        orientation=Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]),
    )


def _joint_frame_id(hand: str, joint_name: str) -> str:
    return f"rkk_{hand}_hand_xr_{joint_name}"


class BridgeNode(Node):
    def __init__(self, stream: HandStream | None = None, **node_kwargs):
        super().__init__("rkk_hand_bridge", **node_kwargs)

        self.declare_parameter("solver_host", constants.SOLVER_HOST)
        self.declare_parameter("solver_port", constants.SOLVER_PORT)
        self.declare_parameter("reconnect_delay_s", constants.RECONNECT_DELAY_S)
        self.declare_parameter("stamp_source", constants.STAMP_SOURCE)
        self.declare_parameter("publish_tf", constants.PUBLISH_TF)
        self.declare_parameter("publish_markers", constants.PUBLISH_MARKERS)
        self.declare_parameter("marker_lifetime_s", constants.MARKER_LIFETIME_S)
        self.declare_parameter("marker_joint_scale", constants.MARKER_JOINT_SCALE)
        self.declare_parameter("marker_max_joint_radius_m", constants.MARKER_MAX_JOINT_RADIUS_M)
        self.declare_parameter("marker_rate_hz", constants.MARKER_RATE_HZ)
        self.declare_parameter("marker_hand_spacing_m", constants.MARKER_HAND_SPACING_M)
        self.declare_parameter("parent_frame_id", constants.PARENT_FRAME_ID)
        self.declare_parameter("calibrate_facing_rad", constants.CALIBRATE_FACING_RAD)
        self.declare_parameter(
            "calibrate_max_disagreement_rad", constants.CALIBRATE_MAX_DISAGREEMENT_RAD
        )

        self._description_pub = self.create_publisher(HandDescription, "~/hand/description", _LATCHED_QOS)
        self._joints_pub = self.create_publisher(HandJoints, "~/hand/joints", _SENSOR_QOS)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._markers_pub = self.create_publisher(MarkerArray, "~/hand/markers", _MARKER_QOS)
        self._calibrate_srv = self.create_service(Trigger, "~/calibrate", self._handle_calibrate)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._warned_no_parent_frame = False
        self._warned_no_marker_frame = False
        self._last_marker_ns = {}  # device_id -> when markers last went out

        # Written by the watch thread and the stream's reader thread, read
        # by the executor's timer and service callbacks.
        self._state_lock = threading.Lock()
        self._published_description = set()
        self._known_hands = {}  # device_id -> hand string, kept across disconnects
        self._unsupported = {}  # device_id -> reason, for hands we cannot read
        self._stamp_sources = {}
        self._last_timestamp_us = {}
        self._yaw_offset = 0.0  # shared across hands; a room-level correction, not per-hand
        self._ever_calibrated = False

        self._stream = stream or HandStream(
            self.get_parameter("solver_host").value,
            self.get_parameter("solver_port").value,
            reconnect_delay_s=self.get_parameter("reconnect_delay_s").value,
        )
        self._stream.on_unsupported_hand = self._on_unsupported_hand
        self._stream.on_stream_error = self._on_stream_error
        self._stream.start()

        self._diagnostics_timer = self.create_timer(
            _DIAGNOSTICS_PERIOD_S, lambda: self._publish_diagnostics(self._stream.hands())
        )

        self._stopping = threading.Event()
        self._watcher = threading.Thread(target=self._watch, daemon=True)
        self._watcher.start()

    def destroy_node(self):
        self._stopping.set()
        self._stream.stop()
        self._watcher.join(timeout=5.0)
        super().destroy_node()

    def _on_stream_error(self, message: str):
        self.get_logger().warning(f"solver stream: {message}")

    def _on_unsupported_hand(self, device_id: int, reason: str):
        with self._state_lock:
            self._unsupported[device_id] = reason
        self.get_logger().warning(f"solved hand {device_id}: {_unsupported_help(reason)}")

    def _watch(self):
        seen = 0
        while not self._stopping.is_set():
            try:
                seen = self._stream.wait(seen, timeout=1.0)
            except TimeoutError:
                continue
            if self._stopping.is_set() or not self.context.ok():
                return
            try:
                self._publish_new_state()
            except Exception:
                # SIGINT can invalidate the context mid-publish, after the
                # check above; anything else is a real failure.
                if self.context.ok():
                    raise
                return

    def _publish_new_state(self):
        hands = self._stream.hands()
        frames = self._stream.latest()

        with self._state_lock:
            gone = [d for d in self._published_description if d not in hands]
            for device_id in gone:
                self._published_description.discard(device_id)
                self._stamp_sources.pop(device_id, None)
                self._last_timestamp_us.pop(device_id, None)
                self._last_marker_ns.pop(device_id, None)
            new = [d for d, _ in hands.items() if d not in self._published_description]
            for device_id, definition in hands.items():
                self._known_hands[device_id] = definition.hand
                self._unsupported.pop(device_id, None)
            self._published_description.update(new)
            dropped_hands = [self._known_hands.get(d) for d in gone]

        if self.get_parameter("publish_markers").value:
            for hand in dropped_hands:
                if hand is not None:
                    self._markers_pub.publish(hand_markers.deletion(hand))

        for device_id in new:
            self._publish_description(hands[device_id])

        for device_id, frame in frames.items():
            definition = hands.get(device_id)
            if definition is None:
                continue
            if self._last_timestamp_us.get(device_id) == frame.timestamp_us:
                continue
            self._last_timestamp_us[device_id] = frame.timestamp_us
            self._publish_joints(definition, frame, sorted(hands))

    def _publish_description(self, definition):
        msg = HandDescription()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.hand = _hand_code(definition.hand)
        msg.device_id = definition.device_id
        msg.timestamp_epoch = definition.timestamp_epoch or ""
        msg.joint_names = list(definition.joint_names)
        msg.joint_radii = list(definition.joint_radii)
        self._description_pub.publish(msg)

    def _publish_joints(self, definition, frame, device_ids: list[int]):
        converter = fc.XrToRosConverter(yaw_rad=self._yaw_offset)
        stamp_source = self._stamp_source_for(frame.device_id)

        receive_time_ns = self.get_clock().now().nanoseconds
        stamp_ns = stamp_source.compute(frame.timestamp_us, definition.timestamp_epoch, receive_time_ns)
        if stamp_ns is None:
            self.get_logger().warning(
                f"hand {frame.device_id}: dropped a frame stamped before the last "
                f"published one ({stamp_source.dropped} so far)",
                throttle_duration_sec=5.0,
            )
            return

        stamp = _stamp_from_ns(stamp_ns)
        converted = [converter.convert(j.position, j.orientation) for j in frame.joints]

        msg = HandJoints()
        msg.header.stamp = stamp
        msg.hand = _hand_code(definition.hand)
        msg.device_id = frame.device_id
        msg.timestamp_us = frame.timestamp_us
        msg.joints = [_to_pose(position, orientation) for position, orientation in converted]
        self._joints_pub.publish(msg)

        if self.get_parameter("publish_tf").value:
            self._broadcast_tf(definition, converted, stamp)

        if self.get_parameter("publish_markers").value:
            self._publish_markers(definition, converted, stamp, device_ids)

    def _stamp_source_for(self, device_id: int) -> StampSource:
        mode = self.get_parameter("stamp_source").value
        with self._state_lock:
            source = self._stamp_sources.get(device_id)
            if source is None or source.mode != mode:
                source = StampSource(mode)
                self._stamp_sources[device_id] = source
            return source

    def _broadcast_tf(self, definition, converted, stamp: TimeMsg):
        parent_frame_id = self.get_parameter("parent_frame_id").value
        if not parent_frame_id:
            if not self._warned_no_parent_frame:
                self.get_logger().warning("publish_tf is set but parent_frame_id is empty; not broadcasting")
                self._warned_no_parent_frame = True
            return

        transforms = []
        for name, (position, orientation) in zip(definition.joint_names, converted):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent_frame_id
            t.child_frame_id = _joint_frame_id(definition.hand, name)
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = position
            t.transform.rotation = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
            transforms.append(t)
        self._tf_broadcaster.sendTransform(transforms)

    def _due_for_markers(self, device_id: int, hand_count: int) -> bool:
        """marker_rate_hz is a budget shared by every connected hand, so
        what RViz has to draw does not grow as gloves are added."""
        rate_hz = self.get_parameter("marker_rate_hz").value
        if rate_hz <= 0.0:
            return True
        period_ns = 1e9 * max(hand_count, 1) / rate_hz
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self._last_marker_ns.get(device_id)
        if last_ns is not None and now_ns - last_ns < period_ns:
            return False
        self._last_marker_ns[device_id] = now_ns
        return True

    def _publish_markers(self, definition, converted, stamp: TimeMsg, device_ids: list[int]):
        if not self._due_for_markers(definition.device_id, len(device_ids)):
            return

        frame_id = self.get_parameter("parent_frame_id").value
        if not frame_id:
            if not self._warned_no_marker_frame:
                self.get_logger().warning(
                    "publish_markers is set but parent_frame_id is empty; not publishing markers"
                )
                self._warned_no_marker_frame = True
            return

        lifetime = Duration(seconds=self.get_parameter("marker_lifetime_s").value).to_msg()
        self._markers_pub.publish(
            hand_markers.build(
                definition.hand,
                definition.joint_names,
                definition.joint_radii,
                converted,
                frame_id,
                stamp,
                lifetime,
                self.get_parameter("marker_joint_scale").value,
                self.get_parameter("marker_max_joint_radius_m").value,
                hand_markers.slot_offset(
                    device_ids.index(definition.device_id),
                    len(device_ids),
                    self.get_parameter("marker_hand_spacing_m").value,
                ),
            )
        )

    def _publish_diagnostics(self, hands):
        with self._state_lock:
            known = list(self._known_hands.items())
            unsupported = list(self._unsupported.items())
            sources = dict(self._stamp_sources)

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        for device_id, hand in known:
            status = DiagnosticStatus()
            status.hardware_id = str(device_id)
            status.name = f"rkk_hand_bridge: {hand} hand"
            if device_id not in hands:
                status.level = DiagnosticStatus.ERROR
                status.message = "disconnected"
            elif not self._ever_calibrated:
                status.level = DiagnosticStatus.WARN
                status.message = "connected, yaw uncalibrated"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "connected"
            source = sources.get(device_id)
            if source is not None:
                status.values = [
                    KeyValue(key="dropped_frames", value=str(source.dropped)),
                    KeyValue(key="clock_resets", value=str(source.clock_resets)),
                ]
            array.status.append(status)

        for device_id, reason in unsupported:
            status = DiagnosticStatus()
            status.hardware_id = str(device_id)
            status.name = f"rkk_hand_bridge: unsupported hand {device_id}"
            status.level = DiagnosticStatus.ERROR
            status.message = _unsupported_help(reason)
            status.values = [KeyValue(key="reason", value=reason)]
            array.status.append(status)

        self._diagnostics_pub.publish(array)

    def _handle_calibrate(self, request, response):
        headings, unreadable = self._wrist_headings()
        if not headings:
            response.success = False
            response.message = unreadable or "no hand data yet"
            return response

        spread = fc.heading_spread(list(headings.values()))
        limit = self.get_parameter("calibrate_max_disagreement_rad").value
        if spread > limit:
            response.success = False
            response.message = (
                f"hands disagree by {math.degrees(spread):.0f} degrees; calibrate with "
                f"every hand facing the same way, or raise "
                f"calibrate_max_disagreement_rad (now {math.degrees(limit):.0f} degrees)"
            )
            return response

        device_id = min(headings)
        facing_rad = self.get_parameter("calibrate_facing_rad").value
        self._yaw_offset = fc.yaw_offset_for_heading(headings[device_id], facing_rad)
        self._ever_calibrated = True
        self._publish_diagnostics(self._stream.hands())

        with self._state_lock:
            hand = self._known_hands.get(device_id, "unknown")
        response.success = True
        response.message = (
            f"calibrated yaw offset to {self._yaw_offset:.3f} rad "
            f"from the {hand} hand (device {device_id})"
        )
        return response

    def _wrist_headings(self) -> tuple[dict[int, float], str | None]:
        hands = self._stream.hands()
        headings, unreadable = {}, None
        for device_id, frame in self._stream.latest().items():
            definition = hands.get(device_id)
            if definition is None or "wrist" not in definition.joint_names:
                unreadable = unreadable or "no wrist joint available"
                continue
            wrist = frame.joints[definition.joint_names.index("wrist")]
            try:
                headings[device_id] = fc.measure_heading(wrist.orientation)
            except ValueError as exc:
                unreadable = str(exc)
        return headings, unreadable


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # A second SIGINT can land here too - e.g. mid-join() while the
        # background threads unwind - not just at rclpy.spin() above.
        # rclpy.shutdown() must still run either way.
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.shutdown()
