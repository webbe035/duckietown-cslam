#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header, Time
from geometry_msgs.msg import *
import numpy as np
import random
import geometry as g
import g2o

R_offset = np.eyes(3)
t_offset = [0.0, 0.0, 0.0]
offset_position = g2o.Isometry3d(R_offset, t_offset)

# t = [0.0, 0.0, 0.055]
# z_angle = 90
# x_angle = 178
# z_angle = np.deg2rad(z_angle)
# x_angle = np.deg2rad(x_angle)
# R_z = g.rotation_from_axis_angle(np.array([0, 0, 1]), z_angle)
# R_x = g.rotation_from_axis_angle(np.array([1, 0, 0]), x_angle)
# R = np.matmul(R_x, R_z)  # verified!
# H_apriltag_to_base = c
# transform = transform * H_apriltag_to_base.inverse()


class Watchtower(object):
    def __init__(self, name, measure_noise, position=None):
        self.measure_noise = measure_noise
        self.name = name
        if(position != None):
            self.position = offset_position * position
        else:
            print("no position given to %s" % self.name)
            self.position = None

    def sees_apriltag(self, apriltag):
        if(apriltag.isinstance(Apriltag)):
            real_transform = self.position.inverse() * apriltag.position
            # apply noise
            return real_transform
        else:
            print("The object is not apriltag")

    def sees_duckiebot(self, duckiebot):
        if(duckiebot.isinstance(Duckiebot)):
            real_transform = self.position.inverse() * duckiebot.position
            # apply noise
            return real_transform
        else:
            print("The object is not apriltag")


class Duckiebot(object):
    def __init__(self, name, measure_noise, initial_position=None):
        self.name = name
        self.measure_noise = measure_noise
        if(initial_position != None):
            self.position = offset_position * initial_position
        else:
            print("no position given to %s" % self.name)
            self.position = None

    def sees_apriltag(self, apriltag):
        if(apriltag.isinstance(Apriltag)):
            real_transform = self.position.inverse() * apriltag.position
            # apply noise
            return real_transform
        else:
            print("The object is not apriltag")


class Apriltag(object):
    def __init__(self, name, position=None):
        self.name = name
        if(position != None):
            self.position = offset_position * position
        else:
            print("no position given to %s" % self.name)
            self.position = None


def create_circle_of_watchtowers(number, radius, height):
    if number < 2:
        print("not enough watchtowers")
    delta_theta = np.pi/number
    theta = 0.0
    list_of_watchtowers = []
    for i in range(number):
        t = [radius * np.cos(theta), radius * np.sin(theta), height]
        x_angle = 5 * np.pi / 6
        z_angle = theta + np.pi
        R_z = g.rotation_from_axis_angle(np.array([0, 0, 1]), z_angle)
        R_x = g.rotation_from_axis_angle(np.array([1, 0, 0]), x_angle)
        R = np.matmul(R_z, R_x)
        position = g2o.Isometry3d(R, t)
        name = "watchtower_%02d" % i
        watchtower = Watchtower(name, 0.0, position)
        list_of_watchtowers.append(watchtower)
        theta += delta_theta
    return list_of_watchtowers


def create_square_of_watchtowers(number, radius, height):
    if number < 2:
        print("not enough watchtowers")
    delta_theta = np.pi/number
    theta = 0.0
    list_of_watchtowers = []
    for i in range(number):
        t = None
        if(theta < np.pi/4 or theta > 7*np.pi/4):
            t = [radius, radius * np.tan(theta), height]
        elif(3*np.pi/4 < theta < 5*np.pi/4):
            t = [-radius, radius * np.tan(theta), height]
        elif(1*np.pi/4 <= theta <= 3*np.pi/4):
            t = [radius / np.tan(theta), radius, height]
        else:
            t = [radius / np.tan(theta), -radius, height]
        x_angle = 5 * np.pi / 6
        z_angle = theta + np.pi
        R_z = g.rotation_from_axis_angle(np.array([0, 0, 1]), z_angle)
        R_x = g.rotation_from_axis_angle(np.array([1, 0, 0]), x_angle)
        R = np.matmul(R_z, R_x)
        position = g2o.Isometry3d(R, t)
        name = "watchtower_%02d" % i
        watchtower = Watchtower(name, 0.0, position)
        list_of_watchtowers.append(watchtower)
        theta += delta_theta

    return list_of_watchtowers


def talker():
    pub = rospy.Publisher('/pose_odometry', TransformStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    seq = 0
    list_of_watchtowers = create_circle_of_watchtowers(6,10,5)
    for watchtower in list_of_watchtowers:
        print(watchtower.position.t)
    
    list_of_watchtowers = create_square_of_watchtowers(6,10,5)
    for watchtower in list_of_watchtowers:
        print(watchtower.position.t)
    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
