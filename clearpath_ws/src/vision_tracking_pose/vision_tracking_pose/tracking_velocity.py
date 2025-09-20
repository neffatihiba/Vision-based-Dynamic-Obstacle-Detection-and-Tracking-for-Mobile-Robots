#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from vision_tracking_pose_msgs.msg import DetectionArray, TrackedPerson, TrackedPeople
from people_msgs.msg import People, Person
import numpy as np
import time
from ultralytics.trackers import BOTSORT, BYTETracker
from ultralytics.utils import yaml_load, IterableSimpleNamespace
from pathlib import Path


class VelocityTrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        self.get_logger().info("VelocityTrackingNode started.")

        # Tracker config
        #config_path = str(Path.home() / "clearpath_ws/src/vision_tracking_pose/vision_tracking_pose/bytetrack.yaml")
        config_path = str(Path.home() / "clearpath_ws/src/vision_tracking_pose/vision_tracking_pose/botsort.yaml")
        cfg = yaml_load(config_path)
        args = IterableSimpleNamespace(**cfg)
        self.tracker = BYTETracker(args=args, frame_rate=13)
        self.get_logger().info(f"Loaded BYTETracker config from: {config_path}")

        # Subscribers
        self.sub_detections = self.create_subscription(
            DetectionArray,
            '/detections_3d',
            #'/detections',
            self.detections_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("Subscribed to /detections_3d")

        # Publishers
        self.pub_tracked_person = self.create_publisher(TrackedPerson, '/TrackedPerson', 10)
        self.pub_tracked_people = self.create_publisher(TrackedPeople, '/TrackedPeople', 10)
        self.pub_people = self.create_publisher(People, '/people', 10)

        # Track state: id -> (position, timestamp)
        self.track_history = {}

    def detections_callback(self, msg: DetectionArray):
        current_time = time.time()
        self.get_logger().info(f" Received {len(msg.detections)} detections at time {current_time:.2f}")

        tracked_people_msg = TrackedPeople()
        tracked_people_msg.header = msg.header

        people_msg = People()
        people_msg.header = msg.header

        for det in msg.detections:
            self.get_logger().info(f" Processing detection ID: {det.id}")

            left_hip = None
            right_hip = None

            if not det.keypoints3d.data:
                self.get_logger().warn(f" Detection {det.id} has no keypoints3d data.")
                continue

            for kp in det.keypoints3d.data:
                self.get_logger().debug(f" Keypoint ID={kp.id}, Point=({kp.point.x:.2f}, {kp.point.y:.2f}, {kp.point.z:.2f})")
                if kp.id == 12:
                    left_hip = kp.point
                elif kp.id == 13:
                    right_hip = kp.point

            if left_hip and right_hip:
                x = (left_hip.x + right_hip.x) / 2
                y = (left_hip.y + right_hip.y) / 2
                z = (left_hip.z + right_hip.z) / 2
            elif left_hip:
                x, y, z = left_hip.x, left_hip.y, left_hip.z
            elif right_hip:
                x, y, z = right_hip.x, right_hip.y, right_hip.z
            else:
                self.get_logger().warn(f"No valid hips found in detection {det.id}, skipping.")
                continue

            current_pos = np.array([x, y, z])
            velocity = Point(x=0.0, y=0.0, z=0.0)

            track_id = det.id or f"person_{det.class_id}"

            if track_id in self.track_history:
                prev_pos, prev_time = self.track_history[track_id]
                dt = current_time - prev_time
                if dt > 0:
                    delta = (current_pos - prev_pos) / dt
                    velocity.x = float(delta[0])
                    velocity.y = float(delta[1])
                    velocity.z = float(delta[2])
                    self.get_logger().debug(f" Calculated velocity for {track_id}: ({velocity.x:.2f}, {velocity.y:.2f}, {velocity.z:.2f})")

            self.track_history[track_id] = (current_pos, current_time)

            speed = np.linalg.norm([velocity.x, velocity.y, velocity.z])
            movement_label = "moving" if speed > 0.05 else "static"

            tracked_person = TrackedPerson()
            tracked_person.name = str(track_id)
            tracked_person.position = Point(x=x, y=y, z=z)
            tracked_person.velocity = velocity
            tracked_person.reliability = 1.0
            tracked_person.tagnames = ["movement", "speed"]
            tracked_person.tags = [movement_label, f"{speed:.3f}"]

            self.pub_tracked_person.publish(tracked_person)
            tracked_people_msg.people.append(tracked_person)

            person = Person()
            person.name = f"track_{track_id}"
            person.position = tracked_person.position
            person.velocity = tracked_person.velocity
            person.reliability = tracked_person.reliability
            people_msg.people.append(person)

            self.get_logger().info(
                f"Published track_{track_id} | pos=({x:.2f}, {y:.2f}, {z:.2f}) "
                f"vel=({velocity.x:.2f}, {velocity.y:.2f}, {velocity.z:.2f}) | status={movement_label}"
            )

        self.pub_tracked_people.publish(tracked_people_msg)
        self.pub_people.publish(people_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
