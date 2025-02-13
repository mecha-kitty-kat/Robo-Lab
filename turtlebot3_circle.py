#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_in_circle():
    # Initialize ROS node
    rospy.init_node('turtlebot3_circle', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Define velocity message
    vel_msg = Twist()
    radius = 3.0  # Circle radius in meters
    linear_speed = 0.5
    angular_speed = linear_speed / radius  # v = r * ω

    vel_msg.linear.x = linear_speed
    vel_msg.angular.z = angular_speed

    rate = rospy.Rate(10)  # 10 Hz

    rospy.loginfo("Moving TurtleBot3 in a circle...")
    while not rospy.is_shutdown():
        velocity_publisher.publish(vel_msg)
    

if __name__ == '__main__':
    try:
        move_in_circle()
    except rospy.ROSInterruptException:
        pass
