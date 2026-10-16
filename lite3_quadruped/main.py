#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Lite3 chassis primitive with hardware and MuJoCo simulation backends."""
from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from pathlib import Path

from robonix_api import Primitive, Ok, Err, Deferred
from robonix_api.ros import RosBackend

from .backends import Backend, create_backend
from .config import ConfigError, parse_config

logging.basicConfig(
    level=os.environ.get("LITE3_LOG_LEVEL", "INFO"),
    format="[lite3] %(message)s",
)
log = logging.getLogger("lite3")

lite3_quadruped = Primitive(id="lite3_quadruped", namespace="robonix/primitive/chassis")

cmd_vel_pub = None
_odom_yaw: float | None = None
_odom_stamp = 0.0
_odom_cv = threading.Condition()
_motion_lock = threading.Lock()

_pkg_root = Path(
    os.environ.get("RBNX_PACKAGE_ROOT", Path(__file__).resolve().parent.parent)
).resolve()
_backend: Backend | None = None

import chassis_pb2  # noqa: E402
import health_pb2  # noqa: E402
import std_msgs_pb2  # noqa: E402
from quadruped_mcp import SetPosture_Request, SetPosture_Response  # noqa: E402


@lite3_quadruped.grpc(
    "robonix/primitive/chassis/move",
    description=(
        "Move or rotate the Lite3 base. Use forward_m for signed forward/backward "
        "distance, rotate_deg for signed in-place yaw rotation (positive is "
        "counter-clockwise/left, negative is clockwise/right), or angular_z with "
        "duration_sec for raw yaw velocity. The robot must already be standing."
    ),
)
def move(req: "chassis_pb2.ExecuteMoveCommand_Request") -> "chassis_pb2.ExecuteMoveCommand_Response":
    global cmd_vel_pub
    if cmd_vel_pub is None:
        return chassis_pb2.ExecuteMoveCommand_Response(
            status=std_msgs_pb2.String(data=json.dumps({"error": "ROS2 not initialized"})),
        )

    msg = req.command
    speed_mps = float(os.environ.get("LITE3_QUADRUPED_SPEED_MPS", "0.2"))
    ang_speed_rps = float(os.environ.get("LITE3_QUADRUPED_ANG_SPEED_RPS", "0.6"))
    default_dur = float(os.environ.get("LITE3_QUADRUPED_CMD_DURATION_SEC", "1.0"))

    from geometry_msgs.msg import Twist  # type: ignore

    with _motion_lock:
        tw = Twist()
        actual_rotate_deg = 0.0
        if float(getattr(msg, "forward_m", 0.0)) != 0.0:
            forward_m = float(msg.forward_m)
            sign = 1.0 if forward_m > 0 else -1.0
            tw.linear.x = sign * speed_mps
            duration = abs(forward_m) / speed_mps
            mode = "forward_m"
            _publish_for_duration(tw, duration)
        elif float(getattr(msg, "rotate_deg", 0.0)) != 0.0:
            rotate_deg = float(msg.rotate_deg)
            ok, actual_rotate_deg, duration, error = _rotate_closed_loop(
                rotate_deg, ang_speed_rps,
            )
            if not ok:
                return chassis_pb2.ExecuteMoveCommand_Response(
                    status=std_msgs_pb2.String(data=json.dumps({
                        "status": "error",
                        "mode": "rotate_deg",
                        "error": error,
                        "requested_rotate_deg": rotate_deg,
                        "actual_rotate_deg": actual_rotate_deg,
                    })),
                )
            mode = "rotate_deg"
        else:
            tw.linear.x = float(getattr(msg, "linear_x", 0.0))
            tw.linear.y = float(getattr(msg, "linear_y", 0.0))
            tw.linear.z = float(getattr(msg, "linear_z", 0.0))
            tw.angular.x = float(getattr(msg, "angular_x", 0.0))
            tw.angular.y = float(getattr(msg, "angular_y", 0.0))
            tw.angular.z = float(getattr(msg, "angular_z", 0.0))
            duration = float(getattr(msg, "duration_sec", 0.0))
            if duration <= 0.0:
                duration = default_dur
            mode = "velocity"
            _publish_for_duration(tw, duration)

    return chassis_pb2.ExecuteMoveCommand_Response(
        status=std_msgs_pb2.String(data=json.dumps({
            "status": "done",
            "mode": mode,
            "duration_sec": duration,
            "linear_x": tw.linear.x,
            "angular_z": tw.angular.z,
            "actual_rotate_deg": actual_rotate_deg,
        })),
    )


def _publish_for_duration(twist, duration: float) -> None:
    """Publish a velocity at 10 Hz and always finish with a stop command."""
    from geometry_msgs.msg import Twist  # type: ignore

    try:
        deadline = time.monotonic() + max(0.1, duration)
        while time.monotonic() < deadline:
            cmd_vel_pub.publish(twist)
            time.sleep(0.1)
    finally:
        cmd_vel_pub.publish(Twist())


def _wrap_angle(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def _on_odom(msg) -> None:
    """Track the latest odometry yaw for closed-loop angle commands."""
    global _odom_yaw, _odom_stamp
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    with _odom_cv:
        _odom_yaw = math.atan2(siny_cosp, cosy_cosp)
        _odom_stamp = time.monotonic()
        _odom_cv.notify_all()


def _rotate_closed_loop(target_deg: float, max_speed: float):
    """Rotate using accumulated odometry yaw, handling wrap at +/-pi."""
    from geometry_msgs.msg import Twist  # type: ignore

    target = math.radians(target_deg)
    tolerance = math.radians(float(os.environ.get(
        "LITE3_ROTATE_TOLERANCE_DEG", "3.0")))
    timeout = max(5.0, abs(target) / max(0.1, max_speed) * 2.5 + 2.0)
    started = time.monotonic()
    with _odom_cv:
        if _odom_yaw is None or started - _odom_stamp > 1.0:
            _odom_cv.wait(timeout=1.0)
        if _odom_yaw is None or time.monotonic() - _odom_stamp > 1.0:
            return False, 0.0, 0.0, "fresh odometry unavailable"
        previous = _odom_yaw

    accumulated = 0.0
    command = Twist()
    try:
        while time.monotonic() - started < timeout:
            with _odom_cv:
                _odom_cv.wait(timeout=0.1)
                if _odom_yaw is None or time.monotonic() - _odom_stamp > 1.0:
                    return False, math.degrees(accumulated), time.monotonic() - started, "odometry became stale"
                current = _odom_yaw
            accumulated += _wrap_angle(current - previous)
            previous = current
            remaining = target - accumulated
            if abs(remaining) <= tolerance:
                return True, math.degrees(accumulated), time.monotonic() - started, ""
            speed = min(max_speed, max(0.15, abs(remaining) * 1.2))
            command.angular.z = math.copysign(speed, remaining)
            cmd_vel_pub.publish(command)
        return False, math.degrees(accumulated), time.monotonic() - started, "rotation timeout"
    finally:
        cmd_vel_pub.publish(Twist())


@lite3_quadruped.mcp("robonix/primitive/chassis/posture")
def set_posture(req: SetPosture_Request) -> SetPosture_Response:
    """Route a posture request to the selected backend."""
    if _backend is None:
        return SetPosture_Response(
            success=False, message="Lite3 backend is not initialized")
    try:
        success, message = _backend.set_posture(req.posture_name)
    except (OSError, RuntimeError) as error:
        success, message = False, f"posture request failed: {error}"
    return SetPosture_Response(success=success, message=message)


# ── chassis telemetry → Robonix health ───────────────────────────────────
CHASSIS_STATUS_TOPIC = "/chassis/status"
HEALTH_STREAM_INTERVAL_S = 0.5
STATUS_STALE_S = 5.0

_status_lock = threading.Lock()
_status: dict = {}
_status_seen = 0.0
_health_stop = threading.Event()


def _on_status(msg) -> None:
    """Cache the latest /chassis/status frame for the health stream."""
    global _status_seen
    with _status_lock:
        _status.update(
            battery_level=float(msg.battery_level),
            is_charging=bool(msg.is_charging),
            error_state=int(msg.error_state),
            basic_state=int(msg.basic_state),
            gait_state=int(msg.gait_state),
            motion_state=int(msg.motion_state),
            task_state=int(msg.task_state),
        )
        _status_seen = time.monotonic()


# ── sensor liveness → Soma component availability ─────────────────────────
# Soma keeps a single authoritative health snapshot per robot: a later
# provider's frame replaces the previous one outright, so one composite
# reporter must cover every component. Liveness for the lidar/imu/camera is
# taken from each device's own data path (topic freshness); client_audio is
# available when the audio bridge is accepting connections; robot_audio when
# the on-board ALSA card audio_driver plays through is present.
LIDAR_CLOUD_TOPIC = "/scanner/cloud"
IMU_TOPIC = "/livox/imu"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"
AUDIO_BRIDGE_PORT = 60002
# Card id in /proc/asound/cards for the on-board USB speaker/mic; matches the
# plughw:Device,0 target configured for the audio_driver primitive.
ROBOT_AUDIO_CARD = "Device"
SENSOR_STALE_S = 5.0

_sensor_lock = threading.Lock()
_sensor_seen: dict[str, float] = {}


def _mark_sensor(name: str) -> None:
    with _sensor_lock:
        _sensor_seen[name] = time.monotonic()


def _sensor_fresh(name: str, now: float) -> bool:
    with _sensor_lock:
        seen = _sensor_seen.get(name)
    return seen is not None and now - seen < SENSOR_STALE_S


def _on_point_cloud(_msg) -> None:
    _mark_sensor("mid360_lidar")


def _on_imu(_msg) -> None:
    _mark_sensor("mid360_imu")


def _on_camera_info(_msg) -> None:
    _mark_sensor("front_camera")


def _audio_bridge_listening() -> bool:
    """True while the audio bridge owns its TCP port (LISTEN or ESTABLISHED)."""
    try:
        with open("/proc/net/tcp", encoding="ascii") as fh:
            next(fh)
            for line in fh:
                fields = line.split()
                if len(fields) < 4 or ":" not in fields[1]:
                    continue
                port = int(fields[1].rsplit(":", 1)[1], 16)
                if port == AUDIO_BRIDGE_PORT and fields[3] in ("0A", "01"):
                    return True
    except (OSError, ValueError):
        return False
    return False


def _robot_audio_card_present() -> bool:
    """True while the on-board USB audio card is enumerated by ALSA."""
    try:
        with open("/proc/asound/cards", encoding="ascii") as fh:
            for line in fh:
                if "[" not in line or "]" not in line:
                    continue
                card_id = line.split("[", 1)[1].split("]", 1)[0].strip()
                if card_id == ROBOT_AUDIO_CARD:
                    return True
    except OSError:
        return False
    return False


def _health_state() -> "health_pb2.HealthState":
    """Project the cached chassis telemetry onto the Lite3 Soma components.

    Component ids match soma.yaml (body/base, body/base/battery). Soma maps a
    non-zero ``body/base/error`` control to an active fault, ``.../online`` to
    component availability, and ``body/base/battery`` to the power source.
    Sensor availability is reported as ``body/base/torso/<component>/online``.
    """
    with _status_lock:
        snapshot = dict(_status)
        age = time.monotonic() - _status_seen if _status_seen else None

    readings = []
    if age is not None:
        online = 1.0 if age < STATUS_STALE_S else 0.0
        readings.append(health_pb2.SensorReading(name="body/base/online", current_a=online))
        readings.append(health_pb2.SensorReading(name="body/base/battery/online", current_a=online))
        if age < STATUS_STALE_S:
            readings.append(health_pb2.SensorReading(
                name="body/base/error", current_a=float(snapshot["error_state"])))
            # Unmeasured battery scalars stay at the -1 sentinel: Soma treats
            # negative values as unavailable, while a protobuf default of 0.0
            # would read as a real (and alarming) 0 V / 0 °C.
            readings.append(health_pb2.SensorReading(
                name="body/base/battery",
                temp_c=-1.0,
                voltage=-1.0,
                current_a=-1.0,
                battery_percent=snapshot["battery_level"]))
    now = time.monotonic()
    for component, online in (
        ("mid360_lidar", _sensor_fresh("mid360_lidar", now)),
        ("mid360_imu", _sensor_fresh("mid360_imu", now)),
        ("front_camera", _sensor_fresh("front_camera", now)),
        ("client_audio", _audio_bridge_listening()),
        ("robot_audio", _robot_audio_card_present()),
    ):
        readings.append(health_pb2.SensorReading(
            name=f"body/base/torso/{component}/online",
            current_a=1.0 if online else 0.0,
        ))
    return health_pb2.HealthState(
        voltage=-1.0,
        charging=bool(snapshot.get("is_charging", False)),
        remaining_s=-1,
        readings=readings,
    )


@lite3_quadruped.grpc(
    "robonix/primitive/health/state",
    description=(
        "Read the latest Lite3 chassis telemetry (battery, charging, fault "
        "code) as a Robonix health frame."
    ),
)
def get_health_state(_request: "health_pb2.GetHealthState_Request") -> "health_pb2.GetHealthState_Response":
    return health_pb2.GetHealthState_Response(state=_health_state())


@lite3_quadruped.grpc(
    "robonix/primitive/health/stream",
    description="Stream Lite3 chassis telemetry into Soma's health collector.",
)
def stream_health_state(_request, context):
    """Yield telemetry frames until the consumer disconnects or shutdown."""
    while context.is_active() and not _health_stop.is_set():
        yield _health_state()
        _health_stop.wait(HEALTH_STREAM_INTERVAL_S)


@lite3_quadruped.on_init
def init(cfg):
    """Select a backend, register common capabilities, and check sentinels."""
    global cmd_vel_pub, _backend

    try:
        runtime = parse_config(cfg)
        _backend = create_backend(runtime, _pkg_root)
        log.info("selected backend=%s", _backend.name)
        _backend.prepare()
    except (ConfigError, OSError, ValueError, RuntimeError) as error:
        if _backend is not None:
            _backend.shutdown()
        _backend = None
        return Err(f"Lite3 backend initialization failed: {error}")

    try:
        from geometry_msgs.msg import Twist  # type: ignore
        from nav_msgs.msg import Odometry  # type: ignore

        cmd_vel_pub = lite3_quadruped.create_publisher(
            "robonix/primitive/chassis/twist_in",
            topic=runtime.twist_in_topic,
            msg_type=Twist,
            qos="reliable",
            declare=False,
        )
        lite3_quadruped.declare_ros2_topic(
            "robonix/primitive/chassis/twist_in",
            runtime.twist_in_topic,
            qos="reliable",
        )

        lite3_quadruped.create_subscription(
            "robonix/primitive/chassis/odom",
            topic=runtime.odom_topic,
            msg_type=Odometry,
            callback=_on_odom,
            qos="best_effort",
            declare=False,
        )
        lite3_quadruped.declare_ros2_topic(
            "robonix/primitive/chassis/odom",
            runtime.odom_topic,
            qos="reliable",
        )

        if runtime.backend == "hardware":
            # Internal subscriber for the transfer bridge's RobotState decode.
            # No chassis-status contract exists, so bypass contract declaration
            # and use the shared ROS backend directly.
            from transfer_interfaces.msg import RobotStatus  # type: ignore

            RosBackend.get().create_subscription(
                RobotStatus, CHASSIS_STATUS_TOPIC, _on_status, qos="best_effort")

            # Liveness taps for the MID-360 lidar/IMU and front camera: only
            # message arrival time matters, contents are ignored.
            from sensor_msgs.msg import CameraInfo, Imu, PointCloud2  # type: ignore

            backend = RosBackend.get()
            backend.create_subscription(PointCloud2, LIDAR_CLOUD_TOPIC, _on_point_cloud, qos="best_effort")
            backend.create_subscription(Imu, IMU_TOPIC, _on_imu, qos="best_effort")
            backend.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, _on_camera_info, qos="best_effort")
            _health_stop.clear()

        for topic in _backend.required_topics():
            if not _backend.wait_for_topic(lite3_quadruped, topic):
                _backend.shutdown()
                _backend = None
                return Deferred(
                    f"{runtime.backend} topic {topic.name} is unavailable")
    except Exception as error:
        _backend.shutdown()
        _backend = None
        cmd_vel_pub = None
        return Err(f"failed to register Lite3 ROS interfaces: {error}")

    try:
        _backend.verify_activity()
        _backend.activate()
    except (OSError, RuntimeError) as error:
        _backend.shutdown()
        _backend = None
        return Deferred(f"{runtime.backend} backend is not ready: {error}")

    return Ok()


@lite3_quadruped.on_shutdown
def shutdown():
    """Stop motion and shut down only the selected backend."""
    global cmd_vel_pub, _backend
    _health_stop.set()
    if cmd_vel_pub is not None:
        try:
            from geometry_msgs.msg import Twist  # type: ignore
            cmd_vel_pub.publish(Twist())
        except Exception as error:
            log.warning("failed to publish shutdown stop command: %s", error)
    if _backend is not None:
        try:
            _backend.shutdown()
        except Exception as error:
            log.warning("backend shutdown failed: %s", error)
    _backend = None
    cmd_vel_pub = None
    return Ok()


if __name__ == "__main__":
    lite3_quadruped.run()
