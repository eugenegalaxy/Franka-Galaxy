#!/usr/bin/env python3
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
