import rclpy
import json,numpy
from numpy import clip
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

import can
from tinymovr import Tinymovr
from tinymovr.iface.can import CAN
from tinymovr.units import get_registry

from math import pi

ureg = get_registry()
amps = ureg.ampere
s = ureg.second
minute = ureg.minute
tick = ureg.tick
rad = ureg.radian
turn = ureg.turn
deg = ureg.degree

class HardwareAbstractionLayer(Node):

    def __init__(self):
        super().__init__('HardwareAbstractionLayer')
        
        # Lecture du fichier de configuration des moteurs
        f = open("/home/vanille/ros2_ws/src/hal/config.json","r")
        self.config = json.load(f)
        f.close()
        
        self.can_bus = can.Bus(bustype='slcan',channel='/dev/ttyACM0',bitrate=1000000)
        self.iface = CAN(self.can_bus)
        for kmotor,motor in self.config['motors'].items():
            if "id_can" in motor :
                motor["tm"]=Tinymovr(node_id=int(motor["id_can"]), iface=self.iface)
                assert(motor["tm"].motor_config.flags == 1)
                motor["offset"] = motor["tm"].encoder_estimates.position
                self.declare_parameter(kmotor+"_max_speed",motor["max_speed"])
                self.declare_parameter(kmotor+"_max_current",motor["max_current"])
                motor["tm"].set_limits(motor["max_speed"]*turn/minute,motor["max_current"]*amps)
                self.declare_parameter(kmotor+"_gain_integrator",motor["gain_integrator"])
                motor["tm"].set_integrator_gains(motor["gain_integrator"])

        
        self.publisherJoint_ = self.create_publisher(JointState, '/vanille/joint_states', 1)
        self.publisherDiag_ = self.create_publisher(DiagnosticStatus, 'diagnostic',1)
        
        self.subscription = self.create_subscription(
            JointState,
            '/vanille/joint_position_cmd',
            self.update_position_cmd,
            1)

        timer_period = 0.01 # seconds
        timer_period_diag = 2 # seconds
        self.timer = self.create_timer(timer_period, self.routine)
        self.timerDiag = self.create_timer(timer_period_diag, self.updateDiagnostic)

    def update_position_cmd(self, msg : JointState):
        for imotor in range(len(msg.name)):
            kmotor = msg.name[imotor]
            if kmotor in self.config['motors']:
                motor = self.config['motors'][kmotor]
                position_target = msg.position[imotor]*rad
                if numpy.isnan(position_target) :
                    motor["tm"].current_control()
                    motor["tm"].set_cur_setpoint(0.0*amps)
                else:
                    position_target = clip(position_target,motor["limit_lower"]*deg, motor["limit_upper"]*deg)
                    if motor["orientation"] == "direct":
                        motor["tm"].position_control()
                        # motor["tm"].set_pos_setpoint(motor["offset"]+position_target*float(motor["ratio"]))
                        motor["tm"].set_pos_setpoint(motor["offset"]+position_target*motor["ratio"])
                    elif motor["orientation"] == "indirect":
                        motor["tm"].position_control()
                        # motor["tm"].set_pos_setpoint(motor["offset"]-position_target*float(motor["ratio"]))
                        motor["tm"].set_pos_setpoint(motor["offset"]-position_target*motor["ratio"])
        

    def read_positions(self):
        msg = JointState()
        msg.header.stamp = super().get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
        for kmotor,motor in self.config['motors'].items():
            msg.name.append(motor["joint_name"])
            if motor["orientation"] == "direct":
                msg.position.append(float((motor["tm"].encoder_estimates.position-motor["offset"])/float(motor["ratio"])))
                msg.velocity.append(motor["tm"].encoder_estimates.velocity.to(rad/s).m/float(motor["ratio"]))
                msg.effort.append(motor["tm"].Iq.estimate.m*float(motor["ratio"]))
            elif motor["orientation"] == "indirect":
                msg.position.append(float(-(motor["tm"].encoder_estimates.position-motor["offset"])/float(motor["ratio"])))
                msg.velocity.append(-motor["tm"].encoder_estimates.velocity.to(rad/s).m/float(motor["ratio"]))
                msg.effort.append(-motor["tm"].Iq.estimate.m*float(motor["ratio"]))
        self.publisherJoint_.publish(msg)
        
    
    def updateDiagnostic(self):
        # tmx.device_info = {"device_id": 99999, "fw_major": 0, "fw_minor": 7, "fw_patch": 1, "temp": 45}
        # tmx.motor_config = {"flags": 1, "R": 200, "pole_pairs": 11, "L": 100}
        msg = DiagnosticStatus()
        msg1 = KeyValue()
        for kmotor,motor in self.config['motors'].items():
            msg.values= []
            msg.hardware_id = kmotor
            msg.name = kmotor
            msg.message = "device_info motor_config"
            for kinfo,info in motor["tm"].device_info.items():
                msg1 = KeyValue()
                msg1.key=kinfo
                msg1.value=str(info)
                msg.values.append(msg1)
            for kinfo,info in motor["tm"].motor_config.items():
                msg1 = KeyValue()
                msg1.key=kinfo
                msg1.value=str(info)
                msg.values.append(msg1)
            self.publisherDiag_.publish(msg)
    
    def routine(self):
        self.read_positions()
    

    def stop(self):
        self.get_logger().info(f'Stopping HAL Node')
        for kmotor,motor in self.config['motors'].items():
            motor["tm"].idle()

def main(args=None):
    print('Hi from hal.')
    rclpy.init(args=args)

    hal_node = HardwareAbstractionLayer()
    try:
        rclpy.spin(hal_node)
    except KeyboardInterrupt:
        pass
    hal_node.stop()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()