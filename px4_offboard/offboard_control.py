#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

import time

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('omega', 0.2)
        self.declare_parameter('altitude', 5.0)
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        # Note: no parameter callbacks are used to prevent sudden inflight changes of radii and omega 
        # which would result in large discontinuities in setpoints
        self.theta = 0.0
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.altitude = self.get_parameter('altitude').value

        # State machine states
        self.state = 'TAKEOFF'
        self.counter = 0

        # Parameters for the movements
        self.altitude = 5.0  # Altitude for takeoff
        self.distance = 1.0  # Distance to move in each direction
        self.position = [0.0, 0.0, 0.0]  # Initial position
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        time.sleep(1)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def publish_vehicle_command(self, command, param1, param2):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(msg)

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            self.move_linear()

    def move_linear(self):
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position = [0.0, 0.0, 0.0]
        trajectory_msg.velocity = [0.0, 0.0, 0.0]
        trajectory_msg.acceleration = [0.0, 0.0, 0.0]
        trajectory_msg.yaw = 3.14/2
        trajectory_msg.yawspeed = 0.0
        if self.state == 'TAKEOFF':
            trajectory_msg.position[2] = -self.altitude
            self.counter += 1
            if self.counter >= 500:
                self.state = 'MOVE_BACKWARD'
                self.counter = 0

        elif self.state == 'MOVE_BACKWARD':
            trajectory_msg.position[2] = -self.altitude
            trajectory_msg.position[1] = -self.distance
            self.counter += 1
            if self.counter >= 250:
                self.state = 'RETURN_START'
                self.counter = 0

        elif self.state == 'RETURN_START':
            trajectory_msg.position[2] = -self.altitude
            trajectory_msg.position[1] = 0.0
            self.counter += 1
            if self.counter >= 250:
                self.state = 'MOVE_RIGHT'
                self.counter = 0

        elif self.state == 'MOVE_RIGHT':
            trajectory_msg.position[2] = -self.altitude
            trajectory_msg.position[0] = self.distance
            self.counter += 1
            if self.counter >= 250:
                self.state = 'MOVE_LEFT'
                self.counter = 0

        elif self.state == 'MOVE_LEFT':
            trajectory_msg.position[2] = -self.altitude
            trajectory_msg.position[0] = -self.distance
            self.counter += 1
            if self.counter >= 250:
                self.state = 'RETURN_0'
                self.counter = 0

        elif self.state == 'RETURN_0':
            trajectory_msg.position[2] = -self.altitude
            trajectory_msg.position[1] = 0.0
            self.counter += 1
            if self.counter >= 250:
                self.state = 'MOVE_UP'
                self.counter = 0

        elif self.state == 'MOVE_UP':
            trajectory_msg.position[2] = -(self.altitude + self.distance)
            self.counter += 1
            if self.counter >= 250:
                self.state = 'MOVE_DOWN'
                self.counter = 0

        elif self.state == 'MOVE_DOWN':
            trajectory_msg.position[2] = -self.altitude
            self.counter += 1
            if self.counter >= 250:
                self.state = 'LAND'
                self.counter = 0

        elif self.state == 'LAND':
            # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
            trajectory_msg.position = [self.radius * np.cos(self.theta), self.radius * np.sin(self.theta), -self.altitude]
            self.theta += self.omega * self.dt

        self.publisher_trajectory.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
