#!/usr/bin/env python3

import math
from typing import Tuple

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


ODOMETRY_TIMEOUT_SEC = 5.0


def quaternion_multiply(
    lhs: Tuple[float, float, float, float], rhs: Tuple[float, float, float, float]
) -> Tuple[float, float, float, float]:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def normalize_quaternion(quaternion: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    sin_r = math.sin(half_roll)
    cos_r = math.cos(half_roll)
    sin_p = math.sin(half_pitch)
    cos_p = math.cos(half_pitch)
    sin_y = math.sin(half_yaw)
    cos_y = math.cos(half_yaw)

    return normalize_quaternion(
        (
            sin_r * cos_p * cos_y - cos_r * sin_p * sin_y,
            cos_r * sin_p * cos_y + sin_r * cos_p * sin_y,
            cos_r * cos_p * sin_y - sin_r * sin_p * cos_y,
            cos_r * cos_p * cos_y + sin_r * sin_p * sin_y,
        )
    )


class FlightMavrosRemapper(Node):
    def __init__(self) -> None:
        super().__init__(
            "flight_mavros_remapping",
            automatically_declare_parameters_from_overrides=True,
        )

        self.odometry_topic = self.declare_parameter(
            "odometry_topic", "/odin1/odometry"
        ).value
        self.mavros_pose_topic = self.declare_parameter(
            "mavros_pose_topic", "/mavros/vision_pose/pose"
        ).value
        sensor_roll_offset_rad = self.declare_parameter(
            "sensor_roll_offset_rad", math.pi
        ).value
        sensor_pitch_offset_rad = self.declare_parameter(
            "sensor_pitch_offset_rad", math.pi / 2.0
        ).value
        sensor_yaw_offset_rad = self.declare_parameter(
            "sensor_yaw_offset_rad", 0.0
        ).value

        self.sensor_to_body_rotation = quaternion_from_rpy(
            sensor_roll_offset_rad, sensor_pitch_offset_rad, sensor_yaw_offset_rad
        )
        self.last_odometry_received_time_ns = self.get_clock().now().nanoseconds
        self.timeout_reported = False
        self.has_timed_out_once = False

        self.mavros_pose_publisher = self.create_publisher(
            PoseStamped, self.mavros_pose_topic, 10
        )
        self.odometry_subscriber = self.create_subscription(
            Odometry, self.odometry_topic, self.odometry_callback, 10
        )
        self.timeout_timer = self.create_timer(0.1, self.check_timeout)

        self.get_logger().info(
            "FlightMavros remapper started: odometry '%s' -> pose '%s', timeout %.1f s."
            % (self.odometry_topic, self.mavros_pose_topic, ODOMETRY_TIMEOUT_SEC)
        )

    def rotate_body_orientation(self, orientation) -> Tuple[float, float, float, float]:
        quat_sensor = (orientation.x, orientation.y, orientation.z, orientation.w)
        return normalize_quaternion(
            quaternion_multiply(quat_sensor, self.sensor_to_body_rotation)
        )

    def check_timeout(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_odometry_received_time_ns < int(ODOMETRY_TIMEOUT_SEC * 1e9):
            return

        if self.timeout_reported:
            return

        message = "No odometry received on '%s' for more than %.1f s." % (
            self.odometry_topic,
            ODOMETRY_TIMEOUT_SEC,
        )

        if not self.has_timed_out_once:
            self.get_logger().error(message)
            self.has_timed_out_once = True
        else:
            self.get_logger().warn(message)

        self.timeout_reported = True

    def odometry_callback(self, msg: Odometry) -> None:
        self.last_odometry_received_time_ns = self.get_clock().now().nanoseconds
        self.timeout_reported = False

        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose.position = msg.pose.pose.position

        x, y, z, w = self.rotate_body_orientation(msg.pose.pose.orientation)
        pose_msg.pose.orientation.x = x
        pose_msg.pose.orientation.y = y
        pose_msg.pose.orientation.z = z
        pose_msg.pose.orientation.w = w

        self.mavros_pose_publisher.publish(pose_msg)


def main() -> None:
    rclpy.init()
    node = FlightMavrosRemapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
