from math import pi
from numpy import clip

import can
from tinymovr import Tinymovr
from tinymovr.iface.can import CAN
from tinymovr.units import get_registry

ureg = get_registry()
Amps = ureg.ampere
s = ureg.second
tick = ureg.tick


MAX_CURRENT = 10.0

class Motor:
    def __init__(self, kv : float, can_id : int):
        self.can_id = can_id
        self.kt=[]
        # self.kt = 30.0 / (pi * kv)
        for n in range(0,len(kv)) :
            self.kt = 30.0 / (pi * kv[n])
        self.init_driver()

    def init_driver(self):
        can_bus = can.Bus(bustype='slcan',channel='/dev/ttyACM0',bitrate=1000000)
        iface = CAN(can_bus)
        self.tm = []
        for n in range(0,len(self.can_id)) :
            self.tm.append(Tinymovr(node_id=self.can_id[n], iface=iface))
            print(self.tm[n].motor_config.flags)     
            assert(self.tm.motor_config.flags == 1)
            resolution = self.tm.motor_config.encoder_ticks
            self.scale = (2 * pi) / resolution
            self.tm.current_control()

    def position(self) -> float:
        ticks = self.tm.encoder_estimates.position
        return ticks * self.scale

    def set_torque_target(self, torque : float):
        iq = torque / self.kt
        iq = clip(iq, -MAX_CURRENT, MAX_CURRENT)
        self.tm.set_cur_setpoint(iq * Amps)
