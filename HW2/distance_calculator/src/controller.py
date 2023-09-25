#!/usr/bin/python3

import rospy
import tf
import rospy
from geometry_msgs.msg import Twist
from distance_calculator.srv import GetNextDestination, GetNextDestinationResponse,GetNextDestinationRequest
from std_srvs.srv import Trigger, TriggerResponse
from nav_msgs.msg import Odometry
import math
import tf
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from math import radians

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        
        # getting specified parameters
        self.linear_speed = rospy.get_param("/controller/linear_vel") # m/s
        self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = radians(rospy.get_param("/controller/goal_angle")) # rad
        self.stop_distance = rospy.get_param("/controller/stop_distance") # m
        self.epsilon = rospy.get_param("/controller/epsilon")
        
        self.next_x = 0
        self.next_y = 0
        self.current_x = 0
        self.current_y = 0 
        
        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        # self.state = self.GO 
        self.state = self.ROTATE
        
        rospy.sleep(1)


    # heading of the robot 
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        return x, y, yaw
    
    
    def get_next_destination(self):
       
        get_next_destination = rospy.ServiceProxy('get_next_destination', GetNextDestination)
        response = get_next_destination(self.current_x, self.current_y)
        self.next_x, self.next_y = response.next_x, response.next_y
        print(f'respose message from get_next_destination:{response.next_x}')
        print(f'respose message from get_next_destination:{response.next_y}')
        return True

    
    def run(self):
        print('***** started *****')
        print(f'*** linear vel = {self.linear_speed} ')
        count=0
        
        sum_error_x = 0
        sum_error_y = 0

        while not rospy.is_shutdown() and count<5:
            

            self.get_next_destination()
            self.next_x = float(self.next_x)
            self.next_y = float(self.next_y)

            self.current_x,self.current_y,prev_angle = self.get_heading()
            
            initial_x, initial_y, _ = self.get_heading()
            
            self.goal_angle = math.atan2(self.next_y - self.current_y, self.next_x-self.current_x)
            
            print('------- ROTATION ------')
            print(f' HEADING BEFORE ROTATION {self.get_heading()}')
            print(f' anggle in degree to go {self.goal_angle}')

         
            remaining = (self.goal_angle - prev_angle)
          
            # print(f'remaining angle before while  {remaining}')
            # print('')
            

            twist = Twist()
            twist.angular.z = self.angular_speed
            # if (self.goal_angle) < 0 else -self.angular_speed
            self.cmd_publisher.publish(twist)

          

            # rotation loop
            while abs(remaining) >= self.epsilon:
                self.current_x, self.current_y, current_angle = self.get_heading()
                self.goal_angle = math.atan2(self.next_y - self.current_y, self.next_x-self.current_x)
                remaining = (self.goal_angle - prev_angle)
                prev_angle = current_angle
                # rospy.sleep(2)

            
            self.cmd_publisher.publish(Twist())
            # print(f'heading  after while  {self.get_heading}')
        
            # print(f'goal angle after while {self.goal_angle}')
            
            print(f' HEADING AFTER ROTATION: {self.get_heading()}')

            rospy.sleep(3)
            
            
            ################################
            


            # self.state = self.GO
            distance_to_goal = self.dictance_calc(self.current_x, self.current_y, self.next_x, self.next_y)

            dist_remaining = distance_to_goal
            prev_x,prev_y,prev_angle = self.get_heading()
            
            print('-------- GO ---------')
            print('-------- INITIAL POSITION ---------')
            print(f'PREVIOUS {prev_x} {prev_y}')

            # print(f'next coordinate before while {self.next_x}  {self.next_y}')

            # print(f'current coordinate before while  { prev_x}  {prev_y}')
            
            twist = Twist()
            twist.linear.x = self.linear_speed
            self.cmd_publisher.publish(twist)
            
          

            # rotation loop
            while dist_remaining >= self.epsilon:
                self.current_x, self.current_y, current_angle = self.get_heading()
                delta = self.dictance_calc(self.current_x, self.current_y, prev_x, prev_y)
                dist_remaining -= delta
                prev_x, prev_y = self.current_x, self.current_y
            
            # print(f'current coordinate after while  {self.current_x}  {self.current_y}')
            self.cmd_publisher.publish(Twist())

            error = (abs(self.next_x - prev_x), abs(self.next_y - prev_y))
            sum_error_x += error[0]
            sum_error_y += error[1]

            
            print('')
            print(f'****************** FIRST(x,y): {initial_x}{initial_y}  GOAL(x,y): {self.next_x} {self.next_y} *************************')
            print(f'****************** ERROR: {error} *************************')

            rospy.sleep(4)


            count+=1
        
        sum_error_x = round(sum_error_x / 5.0 , 2)
        sum_error_y = round(sum_error_y / 5.0 , 2)
        sum_error = (sum_error_x, sum_error_y)

        print(f'****************** SUM ERROR: {sum_error} *************************')



    def dictance_calc(self, x1,y1,x2,y2):
        d = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        return float(d)

if __name__ == "__main__":
    controller = Controller()
    
    controller.run()