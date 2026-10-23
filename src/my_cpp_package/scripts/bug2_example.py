#!/usr/bin/env python3

import math
import random

import rclpy
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


class Bug2Example(Node):
    def __init__(self):
        super().__init__("bug2_example")
        self.declare_parameter("wall_count", 6)
        self.declare_parameter("wall_seed", 23)
        self.pose_publisher = self.create_publisher(PoseStamped, "/bug2/pose", 10)
        self.scan_publisher = self.create_publisher(LaserScan, "/bug2/scan", 10)
        self.obstacle_marker_publisher = self.create_publisher(
            MarkerArray, "/bug2/obstacles", 10
        )
        self.command_subscription = self.create_subscription(
            TwistStamped, "/bug2/cmd_vel", self.command_callback, 10
        )

        self.obstacle_x = 2.0
        self.obstacle_y = 0.0
        self.obstacle_radius = 0.38
        self.walls = self.generate_walls(
            self.get_parameter("wall_count").value,
            self.get_parameter("wall_seed").value,
        )
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.last_command_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.update)

    @staticmethod
    def generate_walls(count, seed):
        rng = random.Random(seed)
        walls = []
        for index in range(max(0, count)):
            side = 1.0 if index % 2 == 0 else -1.0
            center_x = rng.uniform(0.65, 3.55)
            center_y = side * rng.uniform(1.35, 1.85)
            length = rng.uniform(0.45, 0.95)
            angle = rng.uniform(-0.45, 0.45)
            half_dx = 0.5 * length * math.cos(angle)
            half_dy = 0.5 * length * math.sin(angle)
            walls.append(
                (
                    (center_x - half_dx, center_y - half_dy),
                    (center_x + half_dx, center_y + half_dy),
                )
            )
        return walls

    def command_callback(self, command):
        self.linear_speed = command.twist.linear.x
        self.angular_speed = command.twist.angular.z
        self.last_command_time = self.get_clock().now()

    def make_scan(self, stamp):
        scan = LaserScan()
        scan.header.stamp = stamp.to_msg()
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        count = 361
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (count - 1)
        scan.range_min = 0.05
        scan.range_max = 8.0
        scan.ranges = [float("inf")] * count
        scan.intensities = [0.0] * count

        relative_x = self.obstacle_x - self.x
        relative_y = self.obstacle_y - self.y
        center_distance = math.hypot(relative_x, relative_y)
        center_angle = math.atan2(relative_y, relative_x) - self.yaw
        center_angle = math.atan2(math.sin(center_angle), math.cos(center_angle))

        for index in range(count):
            ray_angle = scan.angle_min + index * scan.angle_increment
            nearest = scan.ranges[index]
            angle_difference = math.atan2(
                math.sin(center_angle - ray_angle),
                math.cos(center_angle - ray_angle),
            )
            perpendicular_distance = center_distance * math.sin(angle_difference)
            if abs(perpendicular_distance) >= self.obstacle_radius:
                continue
            along_ray = center_distance * math.cos(angle_difference)
            hit_distance = along_ray - math.sqrt(
                self.obstacle_radius**2 - perpendicular_distance**2
            )
            if scan.range_min <= hit_distance <= min(scan.range_max, nearest):
                scan.ranges[index] = hit_distance

            world_angle = self.yaw + ray_angle
            ray_x = math.cos(world_angle)
            ray_y = math.sin(world_angle)
            for (ax, ay), (bx, by) in self.walls:
                wall_x = bx - ax
                wall_y = by - ay
                denominator = ray_x * wall_y - ray_y * wall_x
                if abs(denominator) < 1e-9:
                    continue
                offset_x = ax - self.x
                offset_y = ay - self.y
                distance = (offset_x * wall_y - offset_y * wall_x) / denominator
                segment_position = (offset_x * ray_y - offset_y * ray_x) / denominator
                if (
                    scan.range_min <= distance <= scan.range_max
                    and 0.0 <= segment_position <= 1.0
                    and distance < scan.ranges[index]
                ):
                    scan.ranges[index] = distance
        return scan

    def publish_obstacle_markers(self, stamp):
        markers = MarkerArray()

        obstacle = Marker()
        obstacle.header.stamp = stamp.to_msg()
        obstacle.header.frame_id = "world"
        obstacle.ns = "bug2_obstacles"
        obstacle.id = 0
        obstacle.type = Marker.CYLINDER
        obstacle.action = Marker.ADD
        obstacle.pose.position.x = self.obstacle_x
        obstacle.pose.position.y = self.obstacle_y
        obstacle.pose.position.z = 0.08
        obstacle.pose.orientation.w = 1.0
        obstacle.scale.x = obstacle.scale.y = 2.0 * self.obstacle_radius
        obstacle.scale.z = 0.16
        obstacle.color.r = 0.95
        obstacle.color.g = 0.18
        obstacle.color.b = 0.08
        obstacle.color.a = 0.95
        markers.markers.append(obstacle)

        for index, (start, end) in enumerate(self.walls, start=1):
            wall = Marker()
            wall.header.stamp = stamp.to_msg()
            wall.header.frame_id = "world"
            wall.ns = "bug2_obstacles"
            wall.id = index
            wall.type = Marker.LINE_LIST
            wall.action = Marker.ADD
            wall.pose.orientation.w = 1.0
            wall.scale.x = 0.16
            wall.color.r = 1.0
            wall.color.g = 0.72
            wall.color.b = 0.05
            wall.color.a = 1.0

            first = Point(x=start[0], y=start[1], z=0.10)
            second = Point(x=end[0], y=end[1], z=0.10)
            wall.points = [first, second]
            markers.markers.append(wall)

        self.obstacle_marker_publisher.publish(markers)

    @staticmethod
    def distance_to_segment(x, y, start, end):
        (ax, ay), (bx, by) = start, end
        dx = bx - ax
        dy = by - ay
        length_squared = dx * dx + dy * dy
        if length_squared == 0.0:
            return math.hypot(x - ax, y - ay)
        fraction = max(
            0.0,
            min(1.0, ((x - ax) * dx + (y - ay) * dy) / length_squared),
        )
        return math.hypot(x - (ax + fraction * dx), y - (ay + fraction * dy))

    def update(self):
        dt = 0.05
        now = self.get_clock().now()
        if (now - self.last_command_time).nanoseconds / 1e9 > 0.5:
            self.linear_speed = 0.0
            self.angular_speed = 0.0

        middle_yaw = self.yaw + self.angular_speed * dt / 2.0
        next_x = self.x + self.linear_speed * dt * math.cos(middle_yaw)
        next_y = self.y + self.linear_speed * dt * math.sin(middle_yaw)
        hits_circle = math.hypot(
            next_x - self.obstacle_x, next_y - self.obstacle_y
        ) < self.obstacle_radius + 0.18
        hits_wall = any(
            self.distance_to_segment(next_x, next_y, start, end) < 0.18
            for start, end in self.walls
        )
        if not hits_circle and not hits_wall:
            self.x = next_x
            self.y = next_y
        else:
            self.linear_speed = 0.0

        self.yaw = math.atan2(
            math.sin(self.yaw + self.angular_speed * dt),
            math.cos(self.yaw + self.angular_speed * dt),
        )

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = "world"
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.pose_publisher.publish(pose)
        self.scan_publisher.publish(self.make_scan(now))
        self.publish_obstacle_markers(now)

def main(args=None):
    rclpy.init(args=args)
    node = Bug2Example()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
