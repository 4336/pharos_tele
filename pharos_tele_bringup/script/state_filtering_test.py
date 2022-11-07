#!/usr/bin/env python

import numpy as np

from math import *

import rospy

import roslib

import copy

import math

from pharos_msgs.msg import StateStamped2016 as VehicleState

from std_msgs.msg import Float32 as Float


class StateFilterTest:

    pub_vel = rospy.Publisher('/test/vel', Float, queue_size=1)

    pub_acc = rospy.Publisher('/test/acc', Float, queue_size=1)

    pub_vel_diff = rospy.Publisher('/test/vel_diff', Float, queue_size=1)

    pub_angle = rospy.Publisher('/test/angle', Float, queue_size=1)

    time = rospy.Time(0)
    time0 = rospy.Time(0)
    count = 40

    acc = 0

    vel_diff = 0

    vel_tuned = 0

    vel = 0

    vel_que = [0] * 40

    angle = 0

    def __init__(self):

        print("init")

    def state_filtering(self):

        derivative = (sum(self.vel_que[-20:-1])-sum(self.vel_que[0:19]))/20

        self.vel_diff = derivative * 5 # == /0.2 # acc[m/s^2]

        self.angle += self.vel_diff / 100

        self.angle *= 0.999

        threshold = 0.0001
        if(self.angle > threshold):
            self.angle -= threshold
        elif(self.angle < -threshold):
            self.angle += threshold
        else:
            self.angle = 0

        self.pub_acc.publish(self.acc*5)

        self.pub_vel_diff.publish(self.vel_diff*5)

        self.pub_vel.publish(self.vel)

        self.pub_angle.publish(self.angle)

    def vehicle_state_callback(self, msg):

        self.time = msg.header.stamp

        self.acc = msg.state.lon_acc

        self.vel = msg.state.velocity

        self.vel_que.append(self.vel)

        self.vel_que.pop(0)

        if(self.time.to_sec() < self.time0.to_sec()):

            self.count = 40

        if(self.count > 0):
            self.count -= 1

        else:            
            self.state_filtering()

        self.time0 = self.time

if __name__ == '__main__':

    rospy.init_node('state_filter_node')

    node = StateFilterTest()

    vehicle_state = rospy.get_param('~vehicle_state_topic', '/vehicle/state2016')

    rospy.Subscriber(vehicle_state, VehicleState, node.vehicle_state_callback, queue_size=1)

    rospy.spin()