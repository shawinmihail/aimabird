#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, Twist


cmd_topic_name = "/aimabird_control/photonCmd"
tm_topic_name = "/aimabird_control/photonTm"
vel_topic_name = "/cmd_vel"
target_topic_name = "/move_base_simple/goal"


class AttitudeController:

    cmd_pub = None
    tm_sub = None

    target_pub = None
    vel_sub = None

    time = None
    points = None
    point_index = None

    r = None
    r0 = None
    v0 = None

    def __init__(self):

        self.tm_sub = rospy.Subscriber(tm_topic_name, Float32MultiArray, self.tm_cb)
        self.cmd_pub = rospy.Publisher(cmd_topic_name, Float32MultiArray, queue_size=10)

        self.target_pub = rospy.Publisher(target_topic_name, PoseStamped, queue_size=10)
        self.vel_sub = rospy.Subscriber(vel_topic_name, Twist, self.vel_cb)

        self.time = rospy.Time.now()
        self.point_index = 0

        self.points = list()
        self.points.append([0., 0., 3.])
        self.points.append([50., -0., 3.])
        self.points.append([0., 0., 3.])
        # self.points.append([45., 8., 3.])
        # self.points.append([100., 0., 3.])


    def tm_cb(self, data):
        self.r = [data.data[0], data.data[1], data.data[2]]

    def vel_cb(self, data):
        self.v0 = [data.linear.x, data.linear.y, 0.]

    def send_target(self):
        msg = PoseStamped()
        msg.pose.position.x = 3.
        msg.pose.position.y = 0.
        msg.pose.position.z = 0.
        msg.pose.orientation.w = 1.
        msg.pose.orientation.x = 0.
        msg.pose.orientation.y = 0.
        msg.pose.orientation.z = 0.
        msg.header.frame_id = "/map"
        msg.header.stamp = rospy.Time.now()
        self.target_pub.publish(msg)

    def main(self):
        self.send_target()
        self.time = rospy.Time.now()

        yaw_rate = 0.0  # rad / s
        self.send_cmd([0.3, 0., 0., 0])  # test yaw rate
        return

        if self.r is None:  # wait telemetry
            print("waiting r")
            return

        if self.v0 is None:
            # self.v0 = self.next_point()
            # print("POINT %s NEXT" % self.v0)
            print("waiting vel")
            return

        self.send_cmd(self.v0)
        print("v0: %s" % self.v0)

    def next_point(self):
        if self.point_index < len(self.points):
            point = self.points[self.point_index]
            self.point_index += 1
            return point
        else:
            return None

    def send_cmd(self, cmd0):
        cmd = Float32MultiArray()
        cmd.data = [cmd0[0], cmd0[1], cmd0[2], cmd0[3]]
        self.cmd_pub.publish(cmd)

    def point_reached(self, radius=0.5):
        dr = np.array(self.r0) - np.array(self.r)
        if np.linalg.norm(dr) < radius:
            return True
        return False

if __name__ == "__main__":
    rospy.init_node("aimbird_example")
    rate = rospy.Rate(20)
    AC = AttitudeController()

    while True:
        AC.main()
        rate.sleep()
