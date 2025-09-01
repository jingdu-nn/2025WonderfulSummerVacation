import lcm
import time
import math
from threading import Thread, Lock
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt

class CtrlRobot:
    def __init__(self):
        self._a = Thread(target=self._b)
        self._c = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self._d = robot_control_cmd_lcmt()
        self._e = robot_control_response_lcmt()
        self._f = Lock()
        self._g = 0
        self._h = 1
        self._i = [0.0, 0.0, 0.0]
        self._j = [0.0, 0.0, 0.0]
        self._k = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._l = [0.0, 0.0]
        self._m = [0.0, 0.0, 0.0]
        self._n = [0.0, 0.0, 0.0]
        self._o = [0.0, 0.0, 0.0]
        self._p = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._q = [0.0, 0.0]
        self._r = 0
        self._s = 0
        self._t = 0
        self._u = None
        self._v = [0.0, 0.0, 0.0]
        self._w = [0.0, 0.0, 0.0]
        self._x = [0.0, 0.0, 0.0]
        self._y = [0.0, 0.0, 0.0]
        self._z = 0.0
        self._aa = 0.0
        self._ab = 0.0
        self._ac = 0.0
        self._ad = 0.0

    def _b(self):
        while self._h:
            self._f.acquire()
            try:
                if self._g > 20:
                    self._c.publish("robot_control_cmd", self._d.encode())
                    self._g = 0
                self._g += 1
            finally:
                self._f.release()
            time.sleep(0.005)

    def start(self):
        self._a.start()

    def send_cmd(self, msg):
        self._f.acquire()
        try:
            self._g = 50
            msg.life_count += 1
            if msg.life_count > 127:
                msg.life_count = 0
            self._c.publish("robot_control_cmd", msg.encode())
        finally:
            self._f.release()

    def stand(self, duration=2000, sleep=2.0):
        self._d.mode = 12
        self._d.gait_id = 0
        self._d.duration = duration
        self._d.life_count = (self._d.life_count + 1) % 128
        self.send_cmd(self._d)
        time.sleep(sleep)

    def get_current_position(self):
        if self._u is None:
            return None
        return [self._u.pos_des[0], self._u.pos_des[1]]

    def reset_odometry_start_position(self):
        if self._u is None:
            return False
        self._v = [self._u.pos_des[0], self._u.pos_des[1], self._u.rpy_des[2]]
        self._w = [0.0, 0.0, 0.0]
        return True

    def get_traveled_distance(self):
        if self._u is None or self._v is None:
            return 0.0
        current_pos = [self._u.pos_des[0], self._u.pos_des[1]]
        dx = current_pos[0] - self._v[0]
        dy = current_pos[1] - self._v[1]
        return math.sqrt(dx*dx + dy*dy)