#!/usr/bin/env python2

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
