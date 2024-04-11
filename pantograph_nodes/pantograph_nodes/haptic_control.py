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


class HapticControl(Node):

    def __init__(self):
        super().__init__('haptic_control_node')
        self.model = PantographModel()  # kinematic model

        # Setup communication
        self.torque_cmd_publisher_ = self.create_publisher(
            Float64MultiArray,
            '/forward_effort_controller/commands',
            5
        )
        self.joint_states_subscription = \
            self.create_subscription(
                JointStateMsg, '/joint_states', self.joint_states_callback, 1)

        # ------------ TODO -------------
        self.publisher_marker_ = None
        # TODO: add your code here
        # hint: create a marker publisher to visualize the haptic environment
        # hint: store the dimension and position of obstacles to use below

        self.publisher_marker_ = self.create_publisher(Marker, '/haptic_env_markers', 10)

        self.env_dict = {}
        self.env_dict['obstacle_position'] = np.array([self.model.a5/2, 0.13])
        self.env_dict['obstacle_radius'] = 0.02
        self.env_dict['p_y_min'] = 0.07
        self.env_dict['p_y_max'] = 0.2
        self.env_dict['p_x_min'] = -0.054
        self.env_dict['p_x_max'] = self.model.a5 + 0.054
        # -------------------------------

        # ------------ TODO -------------
        self.publisher_applied_forces_ = None
        # TODO: add your code here
        # hint: create a geometry_msgs.msg.Wrench publisher to visualize the applied force

        self.publisher_applied_forces_ = \
            self.create_publisher(WrenchMsg, '/applied_forces', 1)
        # -------------------------------

        # Prepare data
        self.last_joint_states_msg = None
        self.F_cmd = None
        self.Ts = 0.002  # sampling period in seconds
        rate_visualization = 30

        # Start timers
        self.timer = self.create_timer(self.Ts, self.update)
        self.timer_visualization = self.create_timer(1/rate_visualization, self.visualization_haptics)

    def update(self):
        # Check that we received joint states before
        if (self.last_joint_states_msg is None):
            return
        self.update_joint_state()
        # *******************************--
        # Update robot state
        # *******************************--
        # compute cartesian state
        self.p = self.model.fk(self.q)
        self.J = self.model.jacobian(self.q)
        # print('p = ', self.p)

        self.p_dot = np.array([0.0, 0.0])

        # ------------ TODO -------------
        # TODO: add your code here
        # hint: compute the cartesian velocity p_dot from self.q_dot using the Jacobian matrix

        self.p_dot = (self.J.T).dot(self.q_dot)
        # -------------------------------

        # *******************************--
        # Compute cartesian force feedback
        # *******************************--

        self.F_cmd = np.array([0.0, 0.0])

        # ------------ TODO -------------
        # TODO: add your code here
        K = 400.0
        if self.p[0] > self.env_dict['p_x_max']:
            self.F_cmd += np.array([self.env_dict['p_x_max'] - self.p[0], 0.0]) * K - self.p_dot * 0.2 * np.sqrt(K)
        if self.p[0] < self.env_dict['p_x_min']:
            self.F_cmd += np.array([self.env_dict['p_x_min'] - self.p[0], 0.0]) * K - self.p_dot * 0.2 * np.sqrt(K)
        if self.p[1] > self.env_dict['p_y_max']:
            self.F_cmd += np.array([0.0, self.env_dict['p_y_max'] - self.p[1]]) * K - self.p_dot * 0.2 * np.sqrt(K)
        if self.p[1] < self.env_dict['p_y_min']:
            self.F_cmd += np.array([0.0, self.env_dict['p_y_min'] - self.p[1]]) * K - self.p_dot * 0.2 * np.sqrt(K)

        K_spheres = 50.0
        if np.linalg.norm(self.p - self.env_dict['obstacle_position']) < self.env_dict['obstacle_radius']:
            vec_radius = np.array(self.p - self.env_dict['obstacle_position'])
            modulus_vec_radius = np.linalg.norm(vec_radius)
            direction_vec_radius = vec_radius / modulus_vec_radius

            modulus_strain = self.env_dict['obstacle_radius'] - modulus_vec_radius
            strain = modulus_strain * direction_vec_radius
            self.F_cmd += strain * K_spheres - self.p_dot * 0.1 * np.sqrt(K_spheres)
        # -------------------------------

        # ----------------------------
        # Compute torque command
        # ----------------------------

        # ------------ TODO -------------
        self.tau_cmd = np.array([0.0, 0.0])
        # TODO: add your code here
        # hint: compute the torque tau_cmd from the force F_cmd

        self.tau_cmd = np.linalg.inv(self.J.T) @ self.F_cmd.reshape((2, 1))
        # -------------------------------

        # ----------------------------
        # Send torque command
        # ----------------------------
        self.send_torque_cmd(self.tau_cmd.reshape((2,)))

    def visualization_haptics(self):
        # Check that update() has been called before
        if (self.F_cmd is None):
            return

        # *******************************
        # Visualization of force feedback
        # *******************************

        # ------------ TODO -------------
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

        # ------------ TODO -------------
        # TODO: add your code here
        # hint: use Rviz marker(s) to visualize the haptic environment

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'sphere_obstacle'
        marker.header.frame_id = 'world'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.env_dict['obstacle_position'][0]
        marker.pose.position.y = self.env_dict['obstacle_position'][1]
        marker.pose.position.z = 0.0
        marker.scale.x = 2.0 * self.env_dict['obstacle_radius']
        marker.scale.y = 2.0 * self.env_dict['obstacle_radius']
        marker.scale.z = 2.0 * self.env_dict['obstacle_radius']
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

    def send_torque_cmd(self, tau_cmd):
        # Do not change this!
        torque_cmd_msg = Float64MultiArray()
        torque_cmd_msg.data = [tau_cmd[0], tau_cmd[1]]
        self.torque_cmd_publisher_.publish(torque_cmd_msg)


def main(args=None):
    # Do not change this!
    rclpy.init(args=args)
    node = HapticControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
