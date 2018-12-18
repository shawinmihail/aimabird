#!/usr/bin/env python

import rospy
import numpy as np
from  std_msgs.msg import Float32MultiArray

cmd_topic_name = "/aimabird_control/photonCmd"
tm_topic_name = "/aimabird_control/photonTm"


class AttitudeController:

    cmd_pub = None
    tm_sub = None
    time = None
    points = None
    point_index = None

    r = None
    r0 = None

    def __init__(self):

        self.tm_sub = rospy.Subscriber(tm_topic_name, Float32MultiArray, self.tm_cb)
        self.cmd_pub = rospy.Publisher(cmd_topic_name, Float32MultiArray, queue_size=10)
        self.time = rospy.Time.now()
        self.point_index = 0

        self.points = list()
        self.points.append([0., 0., 5.])
        self.points.append([3., 0., 5.])

    def tm_cb(self, data):
        self.r = [data.data[0], data.data[1], data.data[2]]

    def main(self):
        self.time = rospy.Time.now()

        if self.r is None:  # wait telemetry
            return

        if self.r0 is None:
            self.r0 = self.next_point()
            print("POINT %s NEXT" % self.r0)

        if self.point_reached():
            print("POINT %s REACHED" % self.r0)
            self.r0 = self.next_point()
            print("POINT %s NEXT" % self.r0)

        if self.r0 is not None:
            self.send_cmd(self.r0)
        else:
            print("END WAY")
            while True:
                pass

        # print(self.r)
        # print(self.r0)

    def next_point(self):
        if self.point_index < len(self.points):
            point = self.points[self.point_index]
            self.point_index += 1
            return point
        else:
            return None

    def send_cmd(self, point):
        cmd = Float32MultiArray()
        cmd.data = [point[0], point[1], point[2]]
        self.cmd_pub.publish(cmd)

    def point_reached(self, radius=0.3):
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