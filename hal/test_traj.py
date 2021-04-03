import rclpy, numpy,time
from rclpy.node import Node

from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform


from math import pi

class TestTraj(Node) :
    def __init__(self):
        super().__init__('TestCmd')
        self.publisher_ = self.create_publisher(MultiDOFJointTrajectory, '/vanille/position_trajectory', 1)
    
        self.animation()

    def animation(self):
        msg = MultiDOFJointTrajectory()
        msg.header.stamp = super().get_clock().now().to_msg()
        msg.joint_names = ["left_front_foot"]
        msg.points=[MultiDOFJointTrajectoryPoint()]
        msg.points[0].time_from_start.sec = 20
        msg.points[0].time_from_start.nanosec = 000000000
        msg.points[0].transforms=[Transform()]
        msg.points[0].transforms[0].translation.x=-0.0
        msg.points[0].transforms[0].translation.y=0.00
        msg.points[0].transforms[0].translation.z=-0.10
        self.publisher_.publish(msg)


def main(args=None):
    print('Hi from test_traj')
    rclpy.init(args=args)

    test_traj_node = TestTraj()
    rclpy.shutdown()

if __name__ == '__main__':
    main()