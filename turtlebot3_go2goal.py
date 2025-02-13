#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt

class TurtleBot3GoToGoal:

    def __init__(self):

        rospy.init_node('turtlebot3_go2goal', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.x = 0
        self.y = 0
        self.theta = 0
        self.rate = rospy.Rate(10)  # 10 Hz

    def odom_callback(self, msg):
        """Extracts position and orientation from Odometry."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def euclidean_distance(self, goal_x, goal_y):
        """Computes Euclidean distance between current pose and goal."""
        return sqrt(pow((goal_x - self.x), 2) + pow((goal_y - self.y), 2))

    def linear_vel(self, goal_x, goal_y, constant=0.2):
        """Proportional control for linear velocity with a minimum threshold."""
        distance = self.euclidean_distance(goal_x, goal_y)
        if distance > 0.2:  # Minimum movement threshold
            return min(constant * distance, 0.5)  # Cap speed to 0.5 m/s
        return 0  # Stop moving when close

    def steering_angle(self, goal_x, goal_y):
        """Computes the angle to the goal."""
        return atan2(goal_y - self.y, goal_x - self.x)

    def angular_vel(self, goal_x, goal_y, constant=1.5):
        """Proportional control for angular velocity."""
        return min(constant * (self.steering_angle(goal_x, goal_y) - self.theta), 1.0)  # Cap speed

    def move2goal(self):
        """Moves the TurtleBot3 to the user-defined goal."""
        goal_x = float(input("Enter X coordinate: "))
        goal_y = float(input("Enter Y coordinate: "))
        distance_tolerance = float(input("Enter distance tolerance: "))

        vel_msg = Twist()

        while self.euclidean_distance(goal_x, goal_y) >= distance_tolerance:
            
            # Compute velocities
            vel_msg.linear.x = self.linear_vel(goal_x, goal_y)
            vel_msg.angular.z = self.angular_vel(goal_x, goal_y)

            # Publish velocity command
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Stop once goal is reached
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        rospy.loginfo("Goal reached!")

if __name__ == '__main__':
    try:
        bot = TurtleBot3GoToGoal()
        bot.move2goal()
    except rospy.ROSInterruptException:
        pass
