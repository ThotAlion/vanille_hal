import rclpy, numpy,time
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import MultiDOFJointTrajectory

class InverseKinematic(Node):

    def __init__(self):
        super().__init__('InverseKinematic')
        self.angle = {}
        self.t0 = {}
        self.x = {}
        self.y = {}
        self.z = {}
        self.duration = {}
        self.angle["hip_x_joint"] = 0.0
        self.angle["hip_y_joint"] = 0.0
        self.angle["leg_y_joint"] = 0.0

        self.publisherJointCmd_ = self.create_publisher(JointState, '/vanille/joint_position_cmd', 1)    
        self.subscriptionJointStates = self.create_subscription(
                JointState,
                '/vanille/joint_states',
                self.update_joint_states,
                1)
        self.subscriptionPositionTrajectory = self.create_subscription(
                MultiDOFJointTrajectory,
                '/vanille/position_trajectory',
                self.update_position_trajectory,
                1)

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.routine)

    def update_joint_states(self, msg : JointState):
        for ijoint in range(len(msg.name)):
            self.angle[msg.name[ijoint]] = msg.position[ijoint]
        

    def update_position_trajectory(self, msg : MultiDOFJointTrajectory) :
        
        for i_joint_name in range(len(msg.joint_names)) :
            joint_name = msg.joint_names[i_joint_name]
            self.t0[joint_name] = msg.header.stamp.sec+msg.header.stamp.nanosec/1000000000.0
            tfs={}
            tfs[joint_name] = msg.points[i_joint_name].transforms
            self.x[joint_name] = []
            self.y[joint_name] = []
            self.z[joint_name] = []
            for tf in tfs[joint_name] :
                self.x[joint_name].append(tf.translation.x)
                self.y[joint_name].append(tf.translation.y)
                self.z[joint_name].append(tf.translation.z)
            self.duration[joint_name]=msg.points[i_joint_name].time_from_start.sec+msg.points[i_joint_name].time_from_start.nanosec/1000000000.0
            print("----toto---")


    def routine(self):
        # geometry of the leg (meters)
        L0 = 0.05365 # Z length between X axis of the hip and Y axis of the hip
        L1 = 0.17457 # Z length between Y axis of the hip and Y axis of the knee
        L2 = 0.17245 # Z length between Y axis of the knee and foot
        Y2 = 0.01851 # Y length between X axis of the hip and foot
        LMIN=0.080
        LMAX=L1+L2
        # computation of left front foot coordinates
        if 'left_front_foot' in self.t0:
            t = time.time()
            #print(self.angle)
            s1=numpy.sin(self.angle["hip_x_joint"])
            s2=numpy.sin(self.angle["hip_y_joint"])
            s23=numpy.sin(self.angle["hip_y_joint"]+self.angle["leg_y_joint"])
            c1=numpy.cos(self.angle["hip_x_joint"])
            c2=numpy.cos(self.angle["hip_y_joint"])
            c23=numpy.cos(self.angle["hip_y_joint"]+self.angle["leg_y_joint"])
            xm = -L1*s2-L2*s23
            ym = L0*s1+L1*c2*s1+L2*c23*s1
            zm = -L0*c1-L1*c2*c1-L2*c23*c1
            #print(xm,ym,zm)

            vt = numpy.linspace(self.t0['left_front_foot'],self.t0['left_front_foot']+self.duration['left_front_foot'],len(self.x['left_front_foot'])+1)
            vx = numpy.concatenate(([xm],self.x['left_front_foot']))
            vy = numpy.concatenate(([ym],self.y['left_front_foot']))
            vz = numpy.concatenate(([zm],self.z['left_front_foot']))
            #print(vt,vx,vy,vz)
            x = numpy.interp(t,vt,vx)
            y = numpy.interp(t,vt,vy)
            z = numpy.interp(t,vt,vz)

            

            # computation of commanded length
            L = numpy.sqrt(x**2+y**2+(z+L0)**2)
            L = numpy.clip(L,LMIN,LMAX)
            #print("--")
            #print(L)

            # computation of angles
            theta1 = numpy.arcsin(y/(numpy.sqrt(x**2+y**2+z**2)))
            #print(theta1)
            theta = -numpy.arcsin(x/L)
            #print(theta)
            theta3 = -numpy.arccos((L**2-L1**2-L2**2)/(2*L1*L2))
            #print(theta3)
            thetaC = numpy.arccos(numpy.clip((L**2+L1**2-L2**2)/(2*L1*L),-1.0,1.0))
            #print(thetaC)
            theta2 = theta+thetaC
            #print(theta2)
            
            msg = JointState()
            msg.name=["hip_x","hip_y","leg_y"]
            msg.position=[theta1,theta2,theta3]
            self.publisherJointCmd_.publish(msg)

        
        
def main(args=None):
    print('Hi inverse kinematic.')
    rclpy.init(args=args)

    ik_node = InverseKinematic()
    try:
        rclpy.spin(ik_node)
    except KeyboardInterrupt:
        pass
    ik_node.stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()