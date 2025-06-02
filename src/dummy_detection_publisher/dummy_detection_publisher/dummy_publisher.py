#!/usr/bin/env python3
"""
Dummy ROS 2 node that publishes synthetic 3D detections and visualization markers
for a set of moving “persons” defined in a YAML configuration.
"""

import os
import math
import yaml
from typing import Any, Dict, List
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Header
from ament_index_python.packages import get_package_share_directory

from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from geometry_msgs.msg import Pose, PoseWithCovariance, Vector3
from visualization_msgs.msg import Marker, MarkerArray


def load_yaml(path: str) -> Any:
    """
    Load and parse a YAML file.

    :param path: File system path to the YAML file
    :return: Parsed YAML content
    """
    with open(path, 'r') as f:
        return yaml.safe_load(f)


class Person:
    """
    Represents a person moving linearly between two waypoints.
    """

    def __init__(
        self,
        person_id: str,
        poses: List[List[float]],
        velocity: float,
        height: float,
        width: float,
    ) -> None:
        """
        :param person_id: Unique identifier
        :param pose1: Start [x, y, z]
        :param pose2: End [x, y, z]
        :param velocity: Movement speed (units/sec)
        :param height: Bounding box height
        :param width: Bounding box width/depth
        """
        self.poses = [np.array(p) for p in poses]
        self.id = person_id
        self.velocity = velocity
        self.height = height
        self.width = width
        self.depth = width
        self.current_pose = self.poses[0]
        self.ipose0 = 0
        self.ipose1 = 1
        self.iinc = 1
        self.velocity_vector: List[float] = [0.0, 0.0, 0.0]

    def update_position(self, time_step: float) -> None:
        """
        Move the person along the line between pose1 and pose2.
        Swap waypoints when target reached.

        :param time_step: Seconds since last update
        """
        if time_step <= 0.0:
            raise ValueError('time_step must be positive')

        direction = self.poses[self.ipose1]-self.poses[self.ipose0]
        dist = np.linalg.norm(direction)
        if dist == 0.0 or self.velocity == 0.0:
            return

        step = [(d / dist) * self.velocity * time_step for d in direction]
        prev = list(self.current_pose)
        self.current_pose = [c + s for c, s in zip(self.current_pose, step)]
        self.velocity_vector = [
            (c - p) / time_step for c, p in zip(self.current_pose, prev)
        ]

        # If within one step of target, swap waypoints

        if np.linalg.norm(self.poses[self.ipose1]-self.current_pose) < self.velocity*time_step:
            self.ipose0 += self.iinc
            self.ipose1 += self.iinc
            if self.ipose1 >= len(self.poses):
                self.iinc = -1
                self.ipose0 = len(self.poses)-1
                self.ipose1 = self.ipose0-1
            if self.ipose1 < 0:
                self.iinc = 1
                self.ipose0 = 0
                self.ipose1 = 1




class Group:
    """
    Collection of Person instances under one group ID.
    """

    def __init__(self, group_id: str, persons: List[Person]) -> None:
        """
        :param group_id: Unique group identifier
        :param persons: List of Person objects
        """
        self.id = group_id
        self.persons = persons


class DummyPublisher(Node):
    """
    Publishes Detection3DArray and MarkerArray at 100 Hz.
    """

    def __init__(self) -> None:
        super().__init__('dummy_publisher')

        default_cfg = os.path.join(
            get_package_share_directory('dummy_detection_publisher'),
            'cfg',
            'persons.yml',
        )
        self.declare_parameter('persons_config', default_cfg)
        cfg = self.get_parameter('persons_config')
        cfg_file = cfg.get_parameter_value().string_value
        data = load_yaml(cfg_file)

        self.persons = {
            p['id']: Person(
                p['id'],
                p['poses'],
                p['velocity'],
                p['height'],
                p['width'],
            )
            for p in data.get('persons', [])
        }
        self.groups = [
            Group(
                g['id'],
                [self.persons[pid] for pid in g.get('persons', [])
                 if pid in self.persons],
            )
            for g in data.get('groups', [])
        ]

        self.d3d_pub = self.create_publisher(
            Detection3DArray, 'detection_vision_3d', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, 'detection_markers', 10
        )

        self.timer_period = 1.0/30.0  # seconds (30 Hz)
        self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self) -> None:
        """
        Update all persons and publish the Detection3DArray and MarkerArray.
        """
        header = self._make_header()
        arr3d = Detection3DArray()
        arr3d.header = header
        markers = MarkerArray()

        for idx, person in enumerate(self.persons.values()):
            person.update_position(self.timer_period)
            det3d, marker = self._build_msgs(idx, person, header)
            arr3d.detections.append(det3d)
            markers.markers.append(marker)

        self.d3d_pub.publish(arr3d)
        self.marker_pub.publish(markers)

    def _make_header(self) -> Header:
        """
        Create a proper std_msgs/Header with current time and 'map' frame.
        """
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = 'map'
        return hdr

    def _build_msgs(self, idx: int, person: Person, header: Header):
        """
        Build Detection3D and Marker for a single person.

        :param idx: Marker ID
        :param person: Person instance
        :param header: Common message header
        :return: (Detection3D, Marker)
        """
        # Compute bounding box extents
        x, y, z = person.current_pose
        hw, hd, hh = person.width / 2, person.depth / 2, person.height / 2
        min_x, max_x = x - hw, x + hw
        min_y, max_y = y - hd, y + hd
        min_z, max_z = z - hh, z + hh

        # Detection3D
        det3d = Detection3D()
        det3d.header = header
        center = Pose()
        center.position.x = (min_x + max_x) / 2.0
        center.position.y = (min_y + max_y) / 2.0
        center.position.z = (min_z + max_z) / 2.0
        center.orientation.w = 1.0
        det3d.bbox.center = center
        size = Vector3()
        size.x = float(max_x - min_x)
        size.y = float(max_y - min_y)
        size.z = float(max_z - min_z)
        det3d.bbox.size = size
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.score = 1.0
        hyp.pose = PoseWithCovariance()
        hyp.pose.pose = center
        det3d.results.append(hyp)

        # Marker
        marker = Marker()
        marker.header = header
        marker.ns = 'detections'
        marker.id = idx
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = (
            'package://dummy_detection_publisher/meshes/lego_man.dae'
        )
        marker.mesh_use_embedded_materials = True
        marker.lifetime = Duration(seconds=0, nanoseconds=200_000_000).to_msg()
        marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
        marker.pose = Pose()
        marker.pose.position = center.position

        vx, vy, _ = person.velocity_vector
        speed = math.hypot(vx, vy)
        yaw = math.atan2(vy, vx) if speed > 1e-6 else 0.0
        half_roll = math.pi / 4
        q_roll = (math.sin(half_roll), 0.0, 0.0, math.cos(half_roll))
        half_yaw = yaw / 2
        q_yaw = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
        x1, y1, z1, w1 = q_yaw
        x2, y2, z2, w2 = q_roll
        qx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        qy = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        qz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        qw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        return det3d, marker


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = DummyPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
