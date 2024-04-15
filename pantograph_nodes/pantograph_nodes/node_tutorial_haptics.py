#!/usr/bin/env python3

# Copyright 2024 ICUBE Laboratory, University of Strasbourg
# License: Apache License, Version 2.0
# Author: Thibault Poignonec (tpoignonec@unistra.fr)

import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState as JointStateMsg
from geometry_msgs.msg import WrenchStamped as WrenchMsg
from pantograph_library import PantographModel
from visualization_msgs.msg import Marker


class NodeTutorialHaptics(Node):
    def __init__(self):
        super().__init__('node_tutorial_haptics')
        self.model = PantographModel()  # kinematic model

        # *********************************
        # Setup communication
        # *********************************
        self.joint_states_subscription = \
            self.create_subscription(
                JointStateMsg, '/joint_states', self.joint_states_callback, 1)

        # ------------ TODO n°1 -------------
        self.torque_cmd_publisher_ = None
        # TODO: add your code here
        # hint: create a publisher to send the force command to the robot using the
        #       function "pub = self.create_publisher(<Type>, <topic_name>, <queue_size>)"

        self.torque_cmd_publisher_ = self.create_publisher(
            Float64MultiArray,
            '/forward_effort_controller/commands',
            5
        )
        # -------------------------------

        # ------------ TODO n°5 -------------
        self.publisher_applied_forces_ = None
        # TODO: add your code here
        # hint: create a geometry_msgs.msg.Wrench publisher to visualize the applied force

        self.publisher_applied_forces_ = \
            self.create_publisher(WrenchMsg, '/applied_forces', 1)
        # -------------------------------

        # ------------ TODO n°9 -------------
        self.publisher_marker_ = None
        # TODO: add your code here
        # hint: create a marker publisher to visualize the haptic environment
        # hint: store the dimension and position of obstacles to use below

        self.publisher_marker_ = self.create_publisher(Marker, '/haptic_env_markers', 10)

        self.env_dict = {}
        self.env_dict['obstacle_position'] = np.array([self.model.a5/2, 0.13])
        self.env_dict['obstacle_radius'] = 0.02
        self.env_dict['obstacle_K'] = 50.0
        self.env_dict['obstacle_D'] = 0.1 * np.sqrt(self.env_dict['obstacle_K'])

        self.env_dict['p_y_min'] = 0.07
        self.env_dict['p_y_max'] = 0.2
        self.env_dict['p_x_min'] = -0.054
        self.env_dict['p_x_max'] = self.model.a5 + 0.054
        self.env_dict['bounds_K'] = 400.0
        self.env_dict['bounds_D'] = 0.2 * np.sqrt(self.env_dict['obstacle_K'])
        # -------------------------------

        # *********************************
        # Prepare data
        # *********************************
        self.last_joint_states_msg = None
        self.F_cmd = None
        self.Ts = 0.002  # sampling period in seconds
        rate_visualization = 30

        # *********************************
        # Start timers
        # *********************************
        self.timer = self.create_timer(self.Ts, self.update)
        self.timer_visualization = self.create_timer(1/rate_visualization, self.visualization_haptics)

    def update(self):
        # Check that we received joint states before
        if (self.last_joint_states_msg is None):
            return
        self.update_joint_state()

        # *********************************
        # Update robot state
        # *********************************
        # compute cartesian state
        self.p = self.model.fk(self.q)
        self.J = self.model.jacobian(self.q)

        # ------------ TODO n° 4 -------------
        # TODO: add your code here
        # hint: compute the cartesian velocity p_dot from self.q_dot using the Jacobian matrix

        self.p_dot = np.array([0.0, 0.0])
        self.p_dot = (self.J).dot(self.q_dot)
        # -------------------------------

        # *********************************
        # Compute cartesian force feedback
        # *********************************

        self.F_cmd = np.array([0.0, 0.0])

        # ------------ TODO n°9 -------------
        # TODO: add your code here
        if self.p[0] > self.env_dict['p_x_max']:
            self.F_cmd += \
                np.array([self.env_dict['p_x_max'] - self.p[0], 0.0]) * self.env_dict['bounds_K'] \
                - self.p_dot * self.env_dict['bounds_D']
        if self.p[0] < self.env_dict['p_x_min']:
            self.F_cmd += \
                np.array([self.env_dict['p_x_min'] - self.p[0], 0.0]) * self.env_dict['bounds_K'] \
                - self.p_dot * self.env_dict['bounds_D']
        if self.p[1] > self.env_dict['p_y_max']:
            self.F_cmd += \
                np.array([0.0, self.env_dict['p_y_max'] - self.p[1]]) * self.env_dict['bounds_K'] \
                - self.p_dot * self.env_dict['bounds_D']
        if self.p[1] < self.env_dict['p_y_min']:
            self.F_cmd += \
                np.array([0.0, self.env_dict['p_y_min'] - self.p[1]]) * self.env_dict['bounds_K'] \
                - self.p_dot * self.env_dict['bounds_D']

        if np.linalg.norm(self.p - self.env_dict['obstacle_position']) < self.env_dict['obstacle_radius']:
            vect_center_sphere_to_p = np.array(self.p - self.env_dict['obstacle_position'])
            dist_center_sphere_to_p = np.linalg.norm(vect_center_sphere_to_p)
            normalized_vect_center_sphere_to_p = vect_center_sphere_to_p / dist_center_sphere_to_p
            strain = (self.env_dict['obstacle_radius'] - dist_center_sphere_to_p) * normalized_vect_center_sphere_to_p
            self.F_cmd += strain * self.env_dict['obstacle_K'] - self.p_dot * self.env_dict['obstacle_D']
        # -------------------------------

        # ------------ TODO n°11 -------------
        # TODO: add your code here
        # goal: X and Y limits
        self.F_cmd += np.array([0.0, 0.0])
        # -------------------------------

        # ------------ TODO n°12 -------------
        # TODO: add your code here
        # goal: sphere obstacle
        self.F_cmd += np.array([0.0, 0.0])
        # -------------------------------

        # ----------------------------
        # Compute torque command
        # ----------------------------

        # ------------ TODO n°3-------------
        self.tau_cmd = np.array([0.0, 0.0])
        # TODO: add your code here
        # hint: compute the torque tau_cmd from the force F_cmd

        self.tau_cmd = np.linalg.inv(self.J) @ self.F_cmd.reshape((2, 1))
        # -------------------------------

        # ----------------------------
        # Send torque command
        # ----------------------------
        torque_cmd_msg = Float64MultiArray()
        torque_cmd_msg.data = self.tau_cmd.reshape((2,)).tolist()

        # ------------ TODO n°2 -------------
        # TODO: add your code here
        # hint: send the torque command message to the robot using
        #       the publisher "self.torque_cmd_publisher_"
        self.torque_cmd_publisher_.publish(torque_cmd_msg)
        # -------------------------------

    def visualization_haptics(self):
        # Check that update() has been called before
        if (self.F_cmd is None):
            return

        # *******************************
        # Visualization of force feedback
        # *******************************

        # ------------ TODO n° 6 -------------
        # TODO: add your code here
        # hint: use Rviz marker to visualize the force stored in "self.F_cmd"

        msg_applied_wrench = WrenchMsg()
        msg_applied_wrench.header.stamp = self.get_clock().now().to_msg()
        msg_applied_wrench.header.frame_id = 'panto_tool0'
        msg_applied_wrench.wrench.force.x = self.F_cmd[0]
        msg_applied_wrench.wrench.force.y = self.F_cmd[1]
        self.publisher_applied_forces_.publish(msg_applied_wrench)
        # -------------------------------

        # *******************************
        # Visualization of environment
        # *******************************

        # ------------ TODO n°10 and n°13 -------------
        # TODO: add your code here
        # hint: use Rviz marker(s) to visualize the haptic environment

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'sphere_obstacle'
        marker.header.frame_id = 'world'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.env_dict['obstacle_position'][0]
        marker.pose.position.y = self.env_dict['obstacle_position'][1]
        marker.pose.position.z = -0.03
        marker.scale.x = 2.0 * self.env_dict['obstacle_radius']
        marker.scale.y = 2.0 * self.env_dict['obstacle_radius']
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 1.0
        self.publisher_marker_.publish(marker)

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'Y_max'
        marker.header.frame_id = 'world'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.model.a5 / 2.0
        marker.pose.position.y = self.env_dict['p_y_max'] + 0.01
        marker.pose.position.z = -0.03
        marker.scale.x = self.env_dict['p_x_max'] - self.env_dict['p_x_min'] + 0.04
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 1.0
        self.publisher_marker_.publish(marker)

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'Y_min'
        marker.header.frame_id = 'world'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.model.a5 / 2.0
        marker.pose.position.y = self.env_dict['p_y_min'] - 0.01
        marker.pose.position.z = -0.03
        marker.scale.x = self.env_dict['p_x_max'] - self.env_dict['p_x_min'] + 0.04
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 1.0
        self.publisher_marker_.publish(marker)

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'X_min'
        marker.header.frame_id = 'world'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.env_dict['p_x_min'] - 0.01
        marker.pose.position.y = self.env_dict['p_y_min'] + 0.5 * (self.env_dict['p_y_max'] - self.env_dict['p_y_min'])
        marker.pose.position.z = -0.03
        marker.scale.x = 0.02
        marker.scale.y = self.env_dict['p_y_max'] - self.env_dict['p_y_min']
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 1.0
        self.publisher_marker_.publish(marker)

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'X_max'
        marker.header.frame_id = 'world'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.env_dict['p_x_max'] + 0.01
        marker.pose.position.y = self.env_dict['p_y_min'] + 0.5 * (self.env_dict['p_y_max'] - self.env_dict['p_y_min'])
        marker.pose.position.z = -0.03
        marker.scale.x = 0.02
        marker.scale.y = self.env_dict['p_y_max'] - self.env_dict['p_y_min']
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 1.0
        self.publisher_marker_.publish(marker)
        # -------------------------------

    def joint_states_callback(self, joint_states_msg):
        # Do not change this!
        self.last_joint_states_msg = joint_states_msg

    def update_joint_state(self):
        # Do not change this!
        jnt_msg = self.last_joint_states_msg
        jnt_position_dict = {}
        jnt_velocity_dict = {}
        jnt_effort_dict = {}
        for idx in range(0, len(jnt_msg.name)):
            qi_name = jnt_msg.name[idx]
            jnt_position_dict[qi_name] = jnt_msg.position[idx]
            jnt_velocity_dict[qi_name] = jnt_msg.velocity[idx]
            jnt_effort_dict[qi_name] = jnt_msg.effort[idx]

        if not ('panto_a1' in jnt_msg.name):
            raise ValueError('Joint data for "panto_a1" not found!')
        if not ('panto_a5' in jnt_msg.name):
            raise ValueError('Joint data for "panto_a5" not found!')

        self.q = np.array([
            jnt_position_dict['panto_a1'],
            jnt_position_dict['panto_a5']
        ])
        self.q_dot = np.array([
            jnt_velocity_dict['panto_a1'],
            jnt_velocity_dict['panto_a5']
        ])
        self.last_tau_cmd = np.array([
            jnt_effort_dict['panto_a1'],
            jnt_effort_dict['panto_a5']
        ])


def main(args=None):
    # Do not change this!
    rclpy.init(args=args)
    node = NodeTutorialHaptics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
