"""
In this script, there is a list of EXAMPLE functions starting with prefix "franka_".
Each functions connects to Desk Web Interface and executes some action. 

IMPOPRTANT: Make sure that "IP_ADDRESS", "LOGIN", and "PASSWORD" are correct.

By using these you can:
1. Open brakes
2. Lock brakes
3. Re-home gripper
4. Execute "Task" (those you create inside web interface, super easy to do, requires no programming)
5. Change Pilot Mode (don't know what it does, but it exists in Web Interface so hey, why not to have it here)
6. Change Guiding Mode (if you are in interactive mode (White light on robot), and move robot by hand, this mode will define how robot moves. Only translation, only rotation, both trans+rot, or "user defined way")
7. Reboot robot (takes ~100 seconds)
8. Shutdown robot.

Author: Jevgenijs Galaktionovs; jgalak16@student.aau.dk (active until June 2021); jga@reallyarobot.com (active always <3)
"""
#!/usr/bin/python3
import json
import hashlib
import base64
import ssl
from time import strftime, sleep
from http.client import HTTPSConnection, HTTPException


def log(message):
    return print(strftime("%H:%M:%S")+"  "+message)


class FrankaWebInterface:
    """API for using Franka methods available in Desk web interface. \
       Uses HTTPS connection and sends requests.

    init args:
        hostname (string): robot IP address used to connect to Franka Desk web interface.
        login (string): User login in Franka Desk web interface.
        password (string): User password in Franka Desk web interface
    """

    def __init__(self, hostname, user, password):
        self._hostname = hostname
        self._user = user
        self._password = password

    def __enter__(self):
        self._client = HTTPSConnection(self._hostname, context=ssl._create_unverified_context())
        self._client.connect()
        encoded_password = self._encode_password(self._user, self._password)
        body_data = json.dumps({'login': self._user, 'password': encoded_password})
        self._client.request('POST', '/admin/api/login',
                             body=body_data,
                             headers={'content-type': 'application/json'})
        self._token = self._client.getresponse().read().decode('utf8')
        return self

    def open_brakes(self):
        self._client.request('POST', '/desk/api/robot/open-brakes',
                             headers={'content-type': 'application/x-www-form-urlencoded',
                                      'Cookie': 'authorization=%s' % self._token})
        return self._client.getresponse()

    def close_brakes(self):
        self._client.request('POST', '/desk/api/robot/close-brakes',
                             headers={'content-type': 'application/x-www-form-urlencoded',
                                      'Cookie': 'authorization=%s' % self._token})
        return self._client.getresponse()

    def reboot(self):
        self._client.request('POST', '/admin/api/reboot')
        return self._client.getresponse()

    def shutdown(self):
        self._client.request('POST', '/admin/api/shutdown')  # NOTE CHECK THIS
        return self._client.getresponse()

    def gripper_homing(self):
        # NOTE: Not tested with other end-effectors other than "Franka Hand".
        self._client.request('POST', '/desk/api/gripper/homing',
                             headers={'content-type': 'application/x-www-form-urlencoded',
                                      'Cookie': 'authorization=%s' % self._token})
        return self._client.getresponse()

    def execute_task(self, task_name):
        body_data = 'id=0_' + task_name
        self._client.request('POST', '/desk/api/execution',
                             body=body_data,
                             headers={'content-type': 'application/x-www-form-urlencoded',
                                      'Cookie': 'authorization=%s' % self._token})
        return self._client.getresponse()

    def pilot_mode(self, mode='one'):
        '''
           Can be 'one' or 'robot'.
        '''
        quot_marks = '"' + mode + '"'  # NOTE string must be inside double quotation marks when passed
        body_data = 'mode=' + quot_marks
        self._client.request('PUT', '/desk/api/navigation/mode',
                             body=body_data,
                             headers={'content-type': 'application/x-www-form-urlencoded',
                                      'Cookie': 'authorization=%s' % self._token})
        return self._client.getresponse()

    def guiding_mode(self, mode='translation'):
        '''
           Can be 'translation', 'rotation', 'free', or 'user'.
        '''
        quot_marks = '"' + mode + '"'  # NOTE string must be inside double quotation marks when passed
        body_data = 'mode=' + quot_marks
        self._client.request('PUT', '/desk/api/robot/guiding/mode',
                             body=body_data,
                             headers={'content-type': 'application/x-www-form-urlencoded',
                                      'Cookie': 'authorization=%s' % self._token})
        return self._client.getresponse()

    def __exit__(self, type, value, traceback):
        self._client.close()

    def _encode_password(self, user, password):
        bs = ','.join([str(b) for b in hashlib.sha256((password + '#' + user + '@franka').encode('utf-8')).digest()])
        return base64.encodebytes(bs.encode('utf-8')).decode('utf-8')


def franka_open_brakes(hostname, login, password):
    with FrankaWebInterface(hostname, login, password) as api:
        try:
            response = api.open_brakes()
            if response.status == 200:
                log('Brakes Unlocked')
            else:
                log('ERROR Opening Brakes. Response status: {}'.format(response.status))
            sleep(5)
        except HTTPException:
            log('ERROR Opening Brakes')


def franka_close_brakes(hostname, login, password):
    with FrankaWebInterface(hostname, login, password) as api:
        try:
            response = api.close_brakes()
            if response.status == 200:
                log('Brakes Locked')
            else:
                log('ERROR Closing Brakes. Response status: {}'.format(response.status))
            sleep(3)
        except HTTPException:
            log('ERROR Closing Brakes')


def franka_gripper_homing(hostname, login, password):
    with FrankaWebInterface(hostname, login, password) as api:
        try:
            response = api.gripper_homing()
            if response.status == 200:
                log('Gripper re-initialized')
            else:
                log('Gripper initialization failed. Response status: {0}'.format(response.status))
        except HTTPException:
            log('ERROR Homing Gripper')


def franka_execute_task(hostname, login, password, TASK_NAME):
    with FrankaWebInterface(hostname, login, password) as api:
        try:
            response = api.execute_task(TASK_NAME)
            if response.status == 200:
                log(" Task '{}' is executed successfully.".format(TASK_NAME))
            else:
                log("ERROR Executing Task '{0}'. Response status: {1}".format(TASK_NAME, response.status))
        except HTTPException:
            log("ERROR Executing Task '{0}'.".format(TASK_NAME))


def franka_pilot_mode(hostname, login, password, MODE):
    with FrankaWebInterface(hostname, login, password) as api:
        try:
            response = api.pilot_mode(mode=MODE)
            if response.status == 200:
                log(" Pilot Mode '{}' is set successfully.".format(MODE))
            else:
                log("ERROR Setting Pilot Mode '{0}'. Response status: {1}".format(MODE, response.status))
        except HTTPException:
            log("ERROR Setting Pilot Mode '{0}'.".format(MODE))


def franka_guiding_mode(hostname, login, password, MODE):
    with FrankaWebInterface(hostname, login, password) as api:
        try:
            response = api.guiding_mode(mode=MODE)
            if response.status == 200:
                log("Guiding Mode '{}' is set successfully.".format(MODE))
            else:
                log("ERROR Setting Guiding Mode '{0}'. Response status: {1}".format(MODE, response.status))
        except HTTPException:
            log("ERROR Setting Guiding Mode '{}'".format(MODE))


def franka_reboot(hostname, login, password):
    with FrankaWebInterface(hostname, login, password) as api:
        try:
            response = api.reboot()
            if response.status == 200:
                log('Franka reboot initialized. Will take ~100 seconds.')
            else:
                log('ERROR Franka reboot. Response status: {}'.format(response.status))
        except HTTPException:
            log('ERROR Franka reboot')


def franka_shutdown(hostname, login, password):
    with FrankaWebInterface(hostname, login, password) as api:
        try:
            response = api.shutdown()
            if response.status == 200:
                log('Franka shutdown initialized.')
            else:
                log('ERROR Franka shutdown. Response status: {}'.format(response.status))
        except HTTPException:
            log('ERROR Franka shutdown')


# Example function calls
HOSTNAME = '172.27.23.65'
LOGIN = 'Panda'
PASSWORD = 'panda1234'

franka_open_brakes(HOSTNAME, LOGIN, PASSWORD)
# franka_gripper_homing(HOSTNAME, LOGIN, PASSWORD)
# franka_close_brakes(HOSTNAME, LOGIN, PASSWORD)

task_name = 'test1'
# franka_execute_task(HOSTNAME, LOGIN, PASSWORD, task_name)

pilot_mode1 = 'robot'  # NOTE must be double quotation string inside a single quote string.
pilot_mode2 = 'one'
# franka_pilot_mode(HOSTNAME, LOGIN, PASSWORD, pilot_mode1)


guiding_mode1 = 'translation'
guiding_mode2 = 'rotation'
guiding_mode3 = 'free'
guiding_mode4 = 'user'
# franka_guiding_mode(HOSTNAME, LOGIN, PASSWORD, guiding_mode1)

# franka_reboot(HOSTNAME, LOGIN, PASSWORD)
# franka_shutdown(HOSTNAME, LOGIN, PASSWORD)
