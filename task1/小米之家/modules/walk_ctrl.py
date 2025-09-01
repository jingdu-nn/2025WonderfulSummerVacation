import time
import math
from .ctrl_robot import CtrlRobot

class _A:
    def __init__(self, _a: CtrlRobot):
        self._b = _a
        
    def _c(self, _d):
        """
        直行控制
        """
        self._b._d.mode = 11
        self._b._d.gait_id = 27
        self._b._d.vel_des = [_d, 0, 0]
        self._b._d.step_height = [0.05, 0.05]
        self._b._d.duration = 0
        self._b._d.life_count = (self._b._d.life_count + 1) % 128
        self._b.send_cmd(self._b._d)
        
    def _e(self):
        """
        停止机器人
        """
        self._b._d.mode = 12
        self._b._d.gait_id = 0
        self._b._d.vel_des = [0, 0, 0]
        self._b._d.duration = 0
        self._b._d.life_count = (self._b._d.life_count + 1) % 128
        self._b.send_cmd(self._b._d)
        
    def _f(self, _g, _h=2.0):
        """
        机器人站立
        """
        self._b._d.mode = 12
        self._b._d.gait_id = 0
        self._b._d.duration = _g
        self._b._d.life_count = (self._b._d.life_count + 1) % 128
        self._b.send_cmd(self._b._d)
        time.sleep(_h)