
#!/usr/bin/python3
import json
import hashlib
import base64
import ssl
from time import strftime, sleep
from http.client import HTTPSConnection, HTTPException


def encode_password(user, password):
    bs = ','.join([str(b) for b in hashlib.sha256((password + '#' + user + '@franka').encode('utf-8')).digest()])
    return base64.encodebytes(bs.encode('utf-8')).decode('utf-8')


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
        encoded_password = encode_password(self._user, self._password)
        self._client.request('POST', '/admin/api/login',
                             body=json.dumps({'login': self._user, 'password': encoded_password}),
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

    def gripper_homing(self):
        # NOTE: Not tested with other end-effectors other than "Franka Hand".
        self._client.request('POST', '/desk/api/gripper/homing',
                             headers={'content-type': 'application/x-www-form-urlencoded',
                                      'Cookie': 'authorization=%s' % self._token})
        return self._client.getresponse()

    def __exit__(self, type, value, traceback):
        self._client.close()


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


# Example function calls
HOSTNAME = '172.27.23.65'
LOGIN = 'Panda'
PASSWORD = 'panda1234'
franka_open_brakes(HOSTNAME, LOGIN, PASSWORD)
franka_gripper_homing(HOSTNAME, LOGIN, PASSWORD)
franka_close_brakes(HOSTNAME, LOGIN, PASSWORD)