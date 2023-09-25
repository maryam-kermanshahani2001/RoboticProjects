#!/usr/bin/python3

from cmath import rect
from matplotlib import pyplot as plt
import numpy as np
import rospy
import tf
import math

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

from math import atan2, pi, radians, sqrt

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        ## self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)
        sub = rospy.Subscriber("/odometry/filtered", Odometry, self.get_heading)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)

        self.twist = Twist()
        
        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO 

        # angle
        self.kp_angle = 2.6
        self.ki_angle = 0.001
        self.kd_angle = 0.9

        #distance
        self.kp_l = 0.75
        self.ki_l = 0.001
        self.kd_l = 0.05
        

    # logarithmic spiral
    def make_logarithmic_spiral(self) :
        logarithmic_spiral = [] 
        a = 0.17
        k = math.tan(a)
        # X , Y = [] , []

        for i in range(150):
            t = i / 20 * math.pi
            dx = a * math.exp(k * t) * math.cos(t)
            dy = a * math.exp(k * t) * math.sin(t)
            # X.append(dx)
            # Y.append(dy)
            logarithmic_spiral.append([dx,dy])
        
        self.path = logarithmic_spiral


    def makeStar(self):
        star = []
        
        X1 = np.linspace(0, 3 , 100)
        Y1 = - (7/3) * X1  + 12
        for i,x in enumerate(X1):
            star.append([x,Y1[i]])

        
        X2 = np.linspace(3, 10 , 100)
        Y2 = np.array([5]*100)
        for i,x in enumerate(X2):
            star.append([x,Y2[i]])

        X3 = np.linspace(10, 4 , 100)
        Y3 = (5/6) * X3  - (10/3)
        for i,x in enumerate(X3):
            star.append([x,Y3[i]])
            
        X4 = np.linspace(4, 7 , 100)
        Y4 = -(3) * X4  + 12
        for i,x in enumerate(X4):
            star.append([x,Y4[i]])


        X5 = np.linspace(7, 0 , 100)
        Y5 = -(4/7) * X5  - 5
        for i,x in enumerate(X5):
            star.append([x,Y5[i]])
        
        
        X6 = np.linspace(0, -7 , 100)
        Y6 = (4/7) * X6  - 5
        for i,x in enumerate(X6):
            star.append([x,Y6[i]])
        

        X7 = np.linspace(-7, -4 , 100)
        Y7 = 3 * X7  + 12
        for i,x in enumerate(X7):
            star.append([x,Y7[i]])

        
        X8 = np.linspace(-4, -10 , 100)
        Y8 = -(5/6) * X8  - (10/3)
        for i,x in enumerate(X8):
            star.append([x,Y8[i]])
        

        X9 = np.linspace(-10, -3 , 100)
        Y9 = np.array([5]*100)
        for i,x in enumerate(X9):
            star.append([x,Y9[i]])


        X10 = np.linspace(-3, 0 , 100)
        Y10 = (7/3) * X10  + 12
        for i,x in enumerate(X10):
            star.append([x,Y10[i]])
        
        self.path = star



    # heading of the robot 
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        return yaw        

    def run(self):

        # step 2
        # self.make_rectangle_path()
        # step 3
        # self.make_archimedean_spiral_path()
        # self.make_logarithmic_spiral()
        self.makeStar()
        # self.make_octagonal()
        # self.make_circles()

        while not rospy.is_shutdown():
            prv_yaw = 0
            for goal in self.path:
                self.cmd_publisher.publish(Twist())
                # rospy.sleep(2)

                next_x = goal[0]
                next_y = goal[1]

                msg = rospy.wait_for_message("/odom" , Odometry) 
                current_x = msg.pose.pose.position.x
                current_y = msg.pose.pose.position.y

                delta_x = next_x - current_x
                delta_y = next_y - current_y

                distance = sqrt(delta_x**2 + delta_y**2)

                sum_distance = 0
                # total_angle = 0
                prv_dist = 0
                # prv_yaw = 0

                while distance > 0.25:
                    current_angle = self.get_heading()
                    msg = rospy.wait_for_message("/odom" , Odometry) 
                    current_x = msg.pose.pose.position.x
                    current_y = msg.pose.pose.position.y

                    delta_x = next_x - current_x
                    delta_y = next_y - current_y

                    goal_angle = atan2(delta_y , delta_x) 

                    if prv_yaw > pi-0.1 and current_angle <= 0:
                        current_angle = 2*pi + current_angle
                    elif prv_yaw < -pi+0.1 and current_angle > 0:
                        current_angle = -2*pi + current_angle
                        
                    if goal_angle < -pi/4 or goal_angle > pi/4:
                        if next_y < 0 and current_y < next_y:
                            goal_angle = -2*pi + goal_angle
                        elif next_y >= 0 and current_y > next_y:
                            goal_angle = 2*pi + goal_angle


                    distance = sqrt(delta_x**2 + delta_y**2)

                    # diff_angle = path_angle - prv_yaw
                    diff_dist = distance - prv_dist

                    p_a = self.kp_angle*(goal_angle - current_angle)
                    self.twist.angular.z = p_a
                    
                    p_l = self.kp_l * distance
                    i_l =  self.ki_l*sum_distance 
                    d_l = self.kd_l*diff_dist
                    
                    pid_l = p_l + i_l + d_l
                    # control_distance = self.kp_d*distance + self.ki_d*total_distance 
                    self.twist.linear.x = min(pid_l,0.1)

                    if self.twist.angular.z <= 0:
                        self.twist.angular.z = max(self.twist.angular.z, -1.5)
                    else:
                        self.twist.angular.z = min(self.twist.angular.z, 1.5)
                    
                    prv_yaw = current_angle
                    self.cmd_publisher.publish(self.twist)

                    rospy.sleep(1)
                    sum_distance += distance
                    prv_dist = distance
                    # total_angle += goal_angle
                    # prv_yaw = path_angle
            # rospy.signal_shutdown()

if __name__ == "__main__":
    controller = Controller()
    controller.run()