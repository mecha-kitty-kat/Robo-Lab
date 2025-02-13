#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

def laser_callback(msg):
    """Callback function for processing LaserScan data."""
    
    ranges = np.array(msg.ranges)  # Convert to NumPy array for easy processing
    valid_ranges = ranges[~np.isnan(ranges)]  # Remove invalid (NaN) values

    # Total number of valid readings
    num_valid_readings = len(valid_ranges)

    # Find closest and farthest points
    if num_valid_readings > 0:
        closest_range = np.min(valid_ranges)
        farthest_range = np.max(valid_ranges)

        # Get angles of closest and farthest points
        closest_index = np.where(ranges == closest_range)[0][0]
        farthest_index = np.where(ranges == farthest_range)[0][0]

        closest_angle = msg.angle_min + closest_index * msg.angle_increment
        farthest_angle = msg.angle_min + farthest_index * msg.angle_increment

        rospy.loginfo("\n📌 LaserScan Data:")
        rospy.loginfo(f"Total valid readings: {num_valid_readings}")
        rospy.loginfo(f"Closest point: {closest_range:.2f} meters at {np.degrees(closest_angle):.2f}°")
        rospy.loginfo(f"Farthest point: {farthest_range:.2f} meters at {np.degrees(farthest_angle):.2f}°\n")
    else:
        rospy.logwarn("⚠️ No valid LaserScan readings received.")

def main():
    rospy.init_node('turtlebot3_laserscan', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.loginfo("Subscribing to /scan topic... Press Ctrl+C to exit.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
