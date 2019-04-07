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
    def __init__(self, name, measure_noise, position = None):
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
            #apply noise 
            return real_transform
        else:
            print("The object is not apriltag")

    def sees_duckiebot(self, duckiebot):
        if(duckiebot.isinstance(Duckiebot)):
            real_transform = self.position.inverse() * duckiebot.position
            #apply noise 
            return real_transform
        else:
            print("The object is not apriltag")

class Duckiebot(object):
    def __init__(self, name, measure_noise, initial_position = None)
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
            #apply noise 
            return real_transform
        else:
            print("The object is not apriltag")

class Apriltag(object):
    def __init__(self, name, position = None):
        self.name = name
        if(position != None):
            self.position = offset_position * position
        else:
            print("no position given to %s" % self.name)
            self.position = None
    

def talker():
    pub = rospy.Publisher('/pose_odometry', TransformStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    seq = 0
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        myTransformStamped = TransformStamped()
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        s0 = random.randint(0, 5)
        node_type = "duckie"
        h.frame_id = "duckie_88"
        # h.frame_id = "%s_%d" % (node_type, s0)

        myTransformStamped.header = h
        d = random.randint(0, 5)

        # rotation = [random.random(), random.random(),
        #             random.random(), random.random()]
        # rotation = rotation/np.linalg.norm(rotation)
        # rotation = Quaternion(
        #     rotation[0], rotation[1], rotation[2], rotation[3])
        # translation = Vector3(
        #     d - s0, d - s0 + random.random(), d - s0 + random.random())
        node_type = random.choice(["duckie", "watchtower"])

        myTransformStamped.transform = Transform(
            Vector3(0, 0, 0), Quaternion(0, 0, 0, 1))
        myTransformStamped.child_frame_id = "duckie_88"
        # myTransformStamped.child_frame_id = "%s_%d" % (node_type, d)

        pub.publish(myTransformStamped)

        rate.sleep()
        seq += 1


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass