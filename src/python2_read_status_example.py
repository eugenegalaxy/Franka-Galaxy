#!/usr/bin/env python2
"""
ROS service to franka_node node that returns Franka robot status: Idle, Move, UserStopped etc...
This script is a trivial Python2 implementation of a ROS service call.
Made as an assignment for the Franka demonstration to deans, November 2020.

IMPOPRTANT: Timeout is not implemented. If franka_node server is not running,
            this script will stuck infinitely.

Author: Jevgenijs Galaktionovs; jgalak16@student.aau.dk (active until June 2021); jga@reallyarobot.com (active always <3)
"""
import rospy
from franka_plugin.srv import bool_key


def read_franka_status():
    rospy.wait_for_service('franka_node/read_status')
    try:
        read_status = rospy.ServiceProxy('franka_node/read_status', bool_key)
        resp = read_status(True)
        return resp.reply
    except rospy.ServiceException as e:
        print("Service call failed: {}".format(e))


if __name__ == "__main__":
    try:
        status = read_franka_status()
        print(status)
    except KeyboardInterrupt:
        quit()
