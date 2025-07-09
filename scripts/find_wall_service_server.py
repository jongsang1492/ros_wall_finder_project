#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from find_wall_pkg.srv import FindWall, FindWallResponse

class FindWallServer:
    def __init__(self):
        rospy.init_node('find_wall_service_server')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.scan_data = None
        self.service = rospy.Service('/find_wall', FindWall, self.handle_find_wall)
        rospy.loginfo("FindWall service is ready.")
        rospy.spin()

    def scan_callback(self, msg):
        self.scan_data = msg

    def handle_find_wall(self, req):
        rate = rospy.Rate(10)
        move = Twist()

        # Step 1: Wait until laser data is received
        while self.scan_data is None:
            rospy.sleep(0.1)

        ranges = self.scan_data.ranges
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment
        total_rays = len(ranges)
        front_index = total_rays // 2

        # Step 2: Find the direction of the shortest distance (closest wall)
        target_dist = min(ranges)
        target_index = ranges.index(target_dist)
        rospy.loginfo(f"▶ Closest distance: {target_dist:.2f}m / Index: {target_index}")

        # Step 3: Rotate until the closest wall is in front of the robot
        while abs(target_index - front_index) > 5 and not rospy.is_shutdown():
            move.linear.x = 0.0
            move.angular.z = 0.3 if target_index > front_index else -0.3
            self.cmd_pub.publish(move)
            rate.sleep()

            ranges = self.scan_data.ranges
            target_index = ranges.index(min(ranges))

        # Step 4: Move forward until the front wall is closer than 0.3m
        while self.scan_data.ranges[front_index] > 0.3 and not rospy.is_shutdown():
            move.linear.x = 0.1
            move.angular.z = 0.0
            self.cmd_pub.publish(move)
            rate.sleep()

        # Stop before rotating to align
        move.linear.x = 0.0
        move.angular.z = 0.0
        self.cmd_pub.publish(move)
        rospy.sleep(1.0)

        # Step 5: Rotate until the closest wall is at index 270 (left side)
        while abs(target_index - 270) > 5 and not rospy.is_shutdown():
            move.angular.z = 0.3 if target_index > 270 else -0.3
            self.cmd_pub.publish(move)
            rate.sleep()

            ranges = self.scan_data.ranges
            target_index = ranges.index(min(ranges))

        # Final stop
        move.angular.z = 0.0
        self.cmd_pub.publish(move)

        rospy.loginfo("Wall alignment complete. wallfound = True")
        return FindWallResponse(wallfound=True)

if __name__ == '__main__':
    FindWallServer()
