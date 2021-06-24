from franka_web_API import *


if __name__ == "__main__":
    # Example function calls.
    HOSTNAME = '172.27.23.65'
    LOGIN = 'Panda'
    PASSWORD = 'panda1234'
    input('Type any char to open brakes: ')
    franka_open_brakes(HOSTNAME, LOGIN, PASSWORD)
    # franka_close_brakes(HOSTNAME, LOGIN, PASSWORD)
    input('Type any char to home gripper: ')
    franka_gripper_homing(HOSTNAME, LOGIN, PASSWORD)

    input('Type any char to executive "task 1": ')
    task_name = 'test1'
    franka_execute_task(HOSTNAME, LOGIN, PASSWORD, task_name)

    pilot_mode1 = 'robot'
    pilot_mode2 = 'one'
    # franka_pilot_mode(HOSTNAME, LOGIN, PASSWORD, pilot_mode1)


    guiding_mode1 = 'translation'
    guiding_mode2 = 'rotation'
    guiding_mode3 = 'free'
    guiding_mode4 = 'user'
    # franka_guiding_mode(HOSTNAME, LOGIN, PASSWORD, guiding_mode1)

    # franka_reboot(HOSTNAME, LOGIN, PASSWORD)
    # franka_shutdown(HOSTNAME, LOGIN, PASSWORD)
except KeyboardInterrupt:
    quit()