import rclpy, numpy,time
from rclpy.node import Node

from sensor_msgs.msg import JointState


from math import pi

class TestCmd(Node) :
    def __init__(self):
        super().__init__('TestCmd')
        self.publisher_ = self.create_publisher(JointState, '/vanille/joint_position_cmd', 1)
    
        timer_period = 2 # seconds
        # self.timer = self.create_timer(timer_period, self.routine)
        self.animation()

    def routine(self):
        msg = JointState()
        msg.name = ["hip_x","hip_y","leg_y"]
        msg.position = [0.0, 0.0, 150.0*pi/180.0]
        self.publisher_.publish(msg)

    def animation(self):

        for x in range(-20,90) :
            msg = JointState()
            msg.name = ["hip_x","hip_y","leg_y"]
            msg.position = [float(x)*pi/180.0, float(x)*pi/180.0, float(x)*pi/180.0]
            self.publisher_.publish(msg)
            time.sleep(0.1)

        for i in range(90,-20,-1) :
            msg = JointState()
            msg.name = ["hip_x","hip_y","leg_y"]
            msg.position = [float(i)*pi/180.0, float(i)*pi/180.0, float(i)*pi/180.0]
            self.publisher_.publish(msg)
            time.sleep(0.1)
        msg = JointState()
        msg.name = ["hip_x","hip_y","leg_y"]
        msg.position = [0.0, 0.0, 0.0]
        self.publisher_.publish(msg)


def main(args=None):
    print('Hi from test_cmd_new')
    rclpy.init(args=args)

    test_cmd_node = TestCmd()
    try:
        rclpy.spin(test_cmd_node)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_cmd_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()