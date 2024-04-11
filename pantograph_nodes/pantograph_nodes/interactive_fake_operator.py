#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import Float64MultiArray

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker
)


class InteractiveFakeOperator(Node):

    def __init__(self):
        super().__init__('interactive_fake_operator_node')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/fake_operator_position',
            5
        )

        # create an interactive marker server on the topic namespace simple_marker
        self.marker_server = InteractiveMarkerServer(self, 'fake_operator')
        self.marker_position = np.array([0.08, 0.18, 0.03])

        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world"
        int_marker.name = "fake_operator_marker"
        int_marker.description = "Current operator's desired position"
        int_marker.scale = 0.05
        int_marker.pose.position.x = self.marker_position[0]
        int_marker.pose.position.y = self.marker_position[1]
        int_marker.pose.position.z = self.marker_position[2]

        # create a marker
        marker = Marker()
        marker.type = Marker.SPHERE
        # marker.always_visible = True
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.
        marker.color.b = 0.
        marker.color.a = 0.6

        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append(marker)
        int_marker.controls.append(visual_control)

        def add_control(x, y, z):
            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = x
            control.orientation.y = y
            control.orientation.z = z
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control)

        add_control(1.0, 0.0, 0.0)
        add_control(0.0, 0.0, 1.0)

        self.marker_server.insert(int_marker)
        self.marker_server.setCallback(int_marker.name, self.process_marker_feedback)

        # 'commit' changes and send to all clients
        self.marker_server.applyChanges()

        self.timer = self.create_timer(0.002, self.update)

    def update(self):
        msg = Float64MultiArray()
        msg.data = [self.marker_position[0], self.marker_position[1]]
        self.publisher_.publish(msg)

    def process_marker_feedback(self, feedback):
        self.marker_position[0] = feedback.pose.position.x
        self.marker_position[1] = feedback.pose.position.y
        self.marker_position[2] = feedback.pose.position.z


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveFakeOperator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
