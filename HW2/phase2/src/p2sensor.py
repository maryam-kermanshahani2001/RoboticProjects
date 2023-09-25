#!/usr/bin/python3

import rospy
import tf

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from phase2.msg import custom_message

from math import pi


class Sensor:
    def __init__(self) -> None:
        rospy.init_node("p2sensor", anonymous=False)

        self.nearest_distance = 0
        self.nearest_angle = 0

        self.pub = rospy.Publisher("ClosestObstacle", custom_message, queue_size=1)
        self.laser_subscriber = rospy.Subscriber(
            "/scan", LaserScan, callback=self.laser_callback
        )

    # checks whether there is an obstacle in front of the robot
    # or not
    def laser_callback(self, msg: LaserScan):
        ranges = msg.ranges
        self.nearest_distance = min(ranges)
        min_index = ranges.index(self.nearest_distance)
        angle = min_index * msg.angle_increment + msg.angle_min

        if angle > pi:
            self.nearest_angle = angle - 2 * pi
        else:
            self.nearest_angle = angle

        obj = custom_message()
        obj.distance = self.nearest_distance
        obj.direction = self.nearest_angle

        self.pub.publish(obj)
        # return angle

    def talker(self):
        while not rospy.is_shutdown():
            pass


if __name__ == "__main__":
    sensor_class = Sensor()

    sensor_class.talker()