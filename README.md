# Hardware abstraction layer for quadruped Vanille
This repository is the ROS2 package taking care about motor control, sensors and kinematics.
All is done from scratch for understanding and discovery purpose about legged robots and ROS2.
The global software architecture is based on ROS2 with a big inspiration from [loki le dev](https://gitlab.com/lokiledev/droid) 
This repository is still under construction and very linked to the robot hardware structure. It is not yet "plug and play"

motor.py, transmission.py are not used
test_cmd.py and test_traj.py are just here to publish msg for test.

## Motor control (vanille_hal node)
The motors used on robot Vanille are Brushless Direct Current MAD Component motors with a Tinymovr driver
- [Tinymovr website](https://tinymovr.com/)
- [MAD Component website](https://mad-motor.com/)
- [Continental belts](https://www.continental-industry.com/fr/solutions/power-transmission/industrial-applications/drive-belts/synchronous-belts/products/product-range/conti-synchroflex-gen3)

To control the motors, we use the [Tinymovr Python Library](https://tinymovr.readthedocs.io/en/latest/api/guide.html#api-reference).

The orders given to the Tnymovr come from a ROS2 topic named /joint_position_cmd with messages of type JointState
/joint_position_cmd :
- name : list of joint names
- position : list of floats with position in radian
- velocity : (Not connected)
- effort : (Not connected)

If the position is set to numpy.NaN, the motor is set to zero hinge moment

On the other side, two topics are published.

The ROS2 topic /joint_states is a topic with messages of type JointStates (sampling period : 10ms)
/joint_states :
- name : list of joint names
- position : list of motor position in radians
- velocity : list of motor velocity in radians/seconds
- effort : list of motor current multiplied per ratio reduction

The ROS2 topic /diagnostic is a topic with messages of type DiagnosticStatus. (sampling period : 2 seconds)
- hardware_id : name of the joint
- name : name of the motor
- message : "device_info motor_config"
- values : list of diagnostic data

Finally, the node takes as an input (later a param) a configuration file config.json which shape is inspired by [pypot](https://github.com/poppy-project/pypot) dynamixel library :
```
"motor_groups": {
        "front_right_leg":["hip_x","hip_y","leg_y"]
    },
    "motors":{
        "hip_x":{
            "id_can": "3",                    //CAN id of the board
            "type" : "Tinymovr",              //name of board product
            "model" : "MAD5005",              //name of motor product
            "motor_kv": "280",                //Theoretical KV of the motor (in rpm/V)
            "ratio":9.0,                      //Ratio of reduction (dimensionless)
            "joint_name":"hip_x_joint",       //Name of the joint in URDF
            "orientation":"indirect",         //orientation of the motor with respect to robot frame ("direct" = anti-clockwise; "indirect" = clockwise according to right-hand rule)
            "limit_upper":45.0,               //upper limit of the motor (in degrees)
            "limit_lower":-40.0,              //lower limit of the motor (in degrees)
            "broken":"false",                 //set true if the motor is broken in emergency case
            "max_speed":1000.0,               //Maximal motor speed (in rmp)
            "max_current":10.0,               //Maximal motor current (in Amps)
            "gain_position":25.0,             //Gain position control
            "gain_velocity":0.00001,          //Gain velocity control
            "gain_integrator":0.001           //Gain velocity integration control (anti-cogging)
        },
        ...
```

## Robot inverse kinematic (inverse_kinematic node)
Under construction