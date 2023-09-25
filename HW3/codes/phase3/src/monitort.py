#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class monitor:
    
    def __init__(self) -> None:
        
        self.path = Path()
        rospy.init_node("monitor" , anonymous=False)
        rospy.Subscriber("/odom" , Odometry , callback = self.odom_callback)
        self.path_publisher = rospy.Publisher("/path" , Path , queue_size = 10)
        
        
    def odom_callback(self, msg : Odometry):
        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)
        
        
if __name__ == "__main__":
    monitor()
    rospy.spin()