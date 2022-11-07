#!/usr/bin/env python

import numpy as np

from math import *

import rospy

import roslib

import copy

import math

from pharos_msgs.msg import StateStamped2016 as VehicleState

from pharos_msgs.msg import MotionCommandStamped3

from pharos_msgs.msg import LaneOffset

from pharos_msgs.msg import SCBADUheader

 

def normalizeSteeringWheel(enc):

    angle = enc / 4096 * 48/40

    theta = theta_deg * 2*math.pi

    return theta

class PharosMaster:

    motion_command_pub = rospy.Publisher('/vehicle/motion_command', MotionCommandStamped3)

    def __init__(self):

        self.velocity = 0

        self.gear = 1

    def vehicle_state_callback(self, msg):

        steer_angle = msg.state.wheel_angle

        gear = msg.state.gear

        new_alpha = msg.state.wheel_angle

        velocity = msg.state.velocity

        time = msg.header.stamp

    def publish_motion_command(self):

        msg = MotionCommandStamped3()

        msg.header.stamp = rospy.Time.now()

        msg.lateral_offset = 0

        msg.curvature = 0

        msg.heading_difference = 0

        msg.vehicle_heading = 0

        msg.wp_path_heading = 0

        msg.planner_path_heading = 0

        msg.velocity_limit = 0

        msg.goal_distance = 0

        msg.inclination = 0

        msg.lateral_offset_origin = 0

        msg.sudden_stop = 0

        msg.mission_state = 0

        self.motion_command_pub.publish(msg)

    def pedals_callback(self, msg):

        self.steering = msg.scbadu.steering

        self.clutch = msg.scbadu.clutch

        self.brake = msg.scbadu.brake

        self.accel = msg.scbadu.accel

        self.down = msg.scbadu.down

        self.up = msg.scbadu.up

        print('%d %.2f %.2f' % (self.gear, self.brake, self.accel))

        if self.up == 1:

            if abs(self.velocity) < 0.1:

                self.gear = 1

        

        if self.down == 1:

            if abs(self.velocity) < 0.1:

                self.gear = 0

        self.acc = self.accel - self.brake*1.1

        if self.gear == 1:

            self.velocity = (self.velocity + self.acc*0.01) * 0.99

            if self.velocity < 0:

                self.velocity = 0

        else:

            self.velocity = (self.velocity - self.acc*0.01) * 0.99

            if self.velocity > 0:

                self.velocity = 0

        self.a = self.steering/4

        self.publish_motion_command()

    # def publish(self, publish_rate):

    #     loop_rate = rospy.Rate(publish_rate)

    #     while not rospy.is_shutdown():

    #         if not self.filter_initialized:

    #             loop_rate.sleep()

    #             continue

    #         self.publish_odom()

    #         try:

    #             loop_rate.sleep()

    #         except rospy.ROSException, e:

    #             if e.message == 'ROS time moved backwards':

    #                 rospy.logwarn("Saw a negative time change, resetting...")

    #                 self.filter_initialized = False

 

    def inspva_callback(self, msg):

        self.azimuth = msg.azimuth

    def corrimudata_callback(self, msg):

        self.yaw_rate = msg.yaw_rate / pi * 180 * 1.5

if __name__ == '__main__':

    rospy.init_node('master_node')

    master = PharosMaster()

    vehicle_state = rospy.get_param('~vehicle_state_topic', '/vehicle/state2016')

    rospy.Subscriber('/master/interface', SCBADUheader , master.pedals_callback, queue_size=1)

    rospy.Subscriber('/vehicle_state', VehicleState, master.vehicle_state_callback, queue_size=1)

    rospy.spin()