#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower_node')
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd = Twist()
        rospy.spin()
    
    def laser_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        
        right_index = int(((-1.57 - angle_min) / angle_increment))  
        front_index = int(((0.0 - angle_min) / angle_increment))    

        right_dist = ranges[right_index]
        front_dist = ranges[front_index]

        rospy.loginfo(f"Right: {right_dist:.2f}, Front: {front_dist:.2f}")


        if front_dist < 0.5:
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = 0.8  
        else:
            
            if right_dist > 0.3:
                self.cmd.linear.x = 0.2
                self.cmd.angular.z = -0.3 
            elif right_dist < 0.2:
                self.cmd.linear.x = 0.2
                self.cmd.angular.z = 0.3   
            else:
                self.cmd.linear.x = 0.2
                self.cmd.angular.z = 0.0   
                
        self.pub.publish(self.cmd)

if __name__ == '__main__':
    WallFollower() 
