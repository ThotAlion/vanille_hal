from dataclasses import dataclass

from math import degrees
@dataclass
class Kinematic:
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0


class Transmission:
    def __init__(self, ratio: float, zero: float = 0.0):
        self.ratio = ratio
        self.zero = zero

    def joint_to_actuator(self, input: Kinematic) -> Kinematic:
        return Kinematic(input.position * self.ratio + self.zero,
                         input.velocity * self.ratio,
                         input.torque / self.ratio)

    def actuator_to_joint(self, input: Kinematic) ->  Kinematic:
        joint = Kinematic((input.position - self.zero) / self.ratio,
                         input.velocity * self.ratio,
                         input.torque * self.ratio)
        #print(f"motor: {degrees(input.position): 0.02f} joint: {degrees(joint.position): 0.02f}")
        return joint
