#!/usr/bin/env python3
"""
A script to run ROS service call through Shell command line and capture the output.
ROS service call is made to franka_node node that returns Franka robot status: Idle, Move, UserStopped etc...
Made as an assignment for the Franka demonstration to deans, November 2020.

Author: Jevgenijs Galaktionovs; jgalak16@student.aau.dk (active until June 2021); jga@reallyarobot.com (active always <3)
"""
import subprocess


def read_franka_status():
    result = subprocess.Popen('rosservice call franka_node/read_status "command: true"',
                              shell=True,
                              stdout=subprocess.PIPE)
    subprocess_return = result.stdout.read()
    return_item = subprocess_return.decode()
    trimmed_item = return_item[8:-2]
    return trimmed_item


if __name__ == "__main__":
    try:
        status = read_franka_status()
        print(status)
    except KeyboardInterrupt:
        quit()
