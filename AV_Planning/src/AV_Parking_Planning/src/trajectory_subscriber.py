#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import pdb

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

## Import controller here or define a callback in the controller itself



def trajectory_subscriber_shutdown():
    print('\n\033[95m' + '*' * 30 + ' ROS Node trajectory_subscriber SHUTDOWN ' + '*' * 30 + '\033[00m\n')

def LQR_control_callback(traj_msg):
    for point in traj_msg.points:
        print(point.positions[0])
    # print(traj_msg)
    # print("HERE")
    # raise NotImplementedError


def main():
    rospy.init_node('trajectory_subscriber', anonymous=True)

    rospy.Subscriber("/planner/trajectory", JointTrajectory, LQR_control_callback)
    rospy.wait_for_message("/planner/trajectory", JointTrajectory)
    rospy.spin()
    r = rospy.Rate(10)
    rospy.on_shutdown(trajectory_subscriber_shutdown)


if __name__ == '__main__':
    main()