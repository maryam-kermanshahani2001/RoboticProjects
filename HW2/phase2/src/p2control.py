#!/usr/bin/python3
# from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import rospy, tf
from nav_msgs.msg import Odometry
from phase2.msg import custom_message
import math

angular_speed = 0.5  # rad/s


class Control:
    def __init__(self) -> None:
        rospy.init_node("p2control", anonymous=False)

        self.linear_speed = 0.4
        self.angular_speed = 0.5

        self.GO, self.ROTATE = 0, 1
        self.state = self.GO
        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def get_obstacle(self) -> custom_message:
        return rospy.wait_for_message("/ClosestObstacle", custom_message)

    def get_heading(self):
        msg = rospy.wait_for_message("/odom", Odometry)

        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )

        return yaw

    def twist_robot(self, s):
        twist = Twist()
        twist.angular.z = 0.0
        twist.linear.x = s
        self.cmd_publisher.publish(twist)

    def run(self):
        while not rospy.is_shutdown():
            speed = Twist()
            rospy.sleep(1)
            twist = Twist()

            twist.linear.x = self.linear_speed
            twist.angular.z = 0
            self.cmd_publisher.publish(twist)
            print("GO")

            while self.state == self.GO:
                msg = self.get_obstacle()
                print(f'NEarest Distance {msg.distance} Nearest Angle {np.rad2deg(msg.direction)}')
                print()
                if msg.distance < 2 and abs(msg.direction) <= math.pi / 2:
                    self.twist_robot(0)
                    self.state = self.ROTATE

            print("ROTATE")
            if self.state == self.ROTATE:
                msg = self.get_obstacle()
                self.twist_robot(0)
                if msg.direction < 0:
                    angle_to_goal = msg.direction + math.pi

                else:
                    angle_to_goal = msg.direction - math.pi

                print(f"rotate  {np.rad2deg(angle_to_goal)}")

                theta = self.get_heading()
                rem = abs(angle_to_goal)

                speed.linear.x = 0.0
                speed.angular.z = angular_speed if angle_to_goal > 0 else -angular_speed
                self.cmd_publisher.publish(speed)

                while rem > 0.001:
                    new_theta = self.get_heading()
                    rem -= abs(abs(new_theta) - abs(theta))
                    theta = new_theta

                self.twist_robot(0)

                rospy.sleep(2)
                self.state = self.GO


if __name__ == "__main__":
    control = Control()
    control.run()