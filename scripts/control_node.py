#!/usr/bin/env python3

# Import ROS for node development
import rospy

# Import Pose message from turtlesim
from turtlesim.msg import Pose

# Import math for conversions
import math

# Import Twist message for control commands
from geometry_msgs.msg import Twist

# Import custom Turtlecontrol message for controller parameters
from robotics_lab1.msg import Turtlecontrol

# Initialize Pose and Turtlecontrol messages
pos_msg = Pose()
cont_msg = Turtlecontrol()

# Define subscriber callback function for position data
def pose_callback(data):
    global pos_msg
    pos_msg = data

# Define subscriber callback function for control parameters
def control_callback(data):
    global cont_msg
    cont_msg = data

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node("turtle_controller", anonymous=True)

    # Subscribe to the pose topic to read position information
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    # Subscribe to the custom control_params topic to read Turtlecontrol data
    control_subscriber = rospy.Subscriber('/turtle1/control_params', Turtlecontrol, control_callback)

    # Declare a publisher to publish velocity commands
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # Set the control loop rate
    loop_rate = rospy.Rate(10)

    # Declare a Twist variable for sending control commands
    vel_msg = Twist()

    # Run the control loop until shutdown
    while not rospy.is_shutdown():
        # Proportional controller equation
        vel_msg.linear.x = cont_msg.kp * (cont_msg.xd - pos_msg.x)

        # Print relevant information for debugging
        print(cont_msg.xd)
        print(cont_msg.kp)
        print(cont_msg.xd - pos_msg.x)
        print("")

        # Publish the velocity command
        velocity_publisher.publish(vel_msg)

        # Sleep to maintain the loop rate
        loop_rate.sleep()
