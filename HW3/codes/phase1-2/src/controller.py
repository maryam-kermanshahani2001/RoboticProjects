#!/usr/bin/python3

import rospy
import tf
import rospy
from geometry_msgs.msg import Twist
from phase1_2.srv import GetNextDestination
from std_srvs.srv import Trigger, TriggerResponse
from nav_msgs.msg import Odometry
import math
import tf
# from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from angles import shortest_angular_distance
from math import radians

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        rospy.on_shutdown(self.shutting_down)

        self.dest_count = 4
       

        self.kp_a = 0.1
        self.ki_a = 0.002
        self.kd_a = 1

        self.kp_l = 0.1
        self.ki_l = 0.002
        self.kd_l = 0

        self.sum_x = 0.0
        self.sum_a = 0.0

        self.prv_err_x = 0.0
        self.prv_err_a = 0.0

        
        self.ep = 0.4
        self.dt = 1

        
        self.next_x = 0
        self.next_y = 0
        self.next_angle = 0

        self.current_x = 0
        self.current_y = 0 
        self.current_angle = 0
        
  
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
        get_next_destination_srv = rospy.ServiceProxy('get_next_destination', GetNextDestination)
        response = get_next_destination_srv(self.current_x, self.current_y)
        self.next_x, self.next_y = response.next_x, response.next_y
       
        print(f'respose message from get_next_destination:{response.next_x}')
        print(f'respose message from get_next_destination:{response.next_y}')
        return True

    def shutting_down(self):
        rospy.loginfo('Stopping the robot')
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


    
    def run(self):
        print('***** started *****')
        count=0
        rospy.sleep(1)
        prv_rotation = 0

        while not rospy.is_shutdown() and count<self.dest_count:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_publisher.publish(twist)

            self.current_x, self.current_y, self.current_angle = self.get_heading()

            self.get_next_destination()
            self.next_x = float(self.next_x)
            self.next_y = float(self.next_y)
            self.sum_x = 0.0
            self.sum_a = 0.0
            self.prv_err_a = 0.0
            self.prv_err_x = 0.0
            # last_angle = 0.0

            dist = self.dictance_calc(self.next_x, self.next_y, self.current_x, self.current_y)

            rate = rospy.Rate(1/self.dt)

            rospy.loginfo(f' Next Station {self.next_x} {self.next_y}')

            while(dist > self.ep):
                self.current_x, self.current_y, self.current_angle = self.get_heading()
                           
                next_angle = math.atan2(self.next_y - self.current_y, self.next_x-self.current_x)
                if next_angle < -math.pi/4 or next_angle > math.pi/4:
                    if self.next_y < 0 and self.current_y < self.next_y:
                        next_angle = -2*math.pi + next_angle
                    elif self.next_y >= 0 and self.current_y > self.next_y:
                        next_angle = 2*math.pi + next_angle
                if prv_rotation > math.pi-0.1 and self.current_angle <= 0:
                    self.current_angle = 2*math.pi + self.current_angle
                elif prv_rotation < -math.pi+0.1 and self.current_angle > 0:
                    self.current_angle = -2*math.pi + self.current_angle
                
                err_a = next_angle - self.current_angle
                pa = self.kp_a * err_a
                self.sum_a +=  (err_a  * self.dt)
                sum_a = self.ki_a * self.sum_a
                da = self.kd_a * (err_a - self.prv_err_a) / self.dt

                PID_a = pa + sum_a + da            
           
                err_x = self.dictance_calc(self.next_x, self.next_y, self.current_x, self.current_y)
                dist = err_x
                
                px = self.kp_l * err_x
                self.sum_x +=  ( err_x * self.dt)
                sum_x = self.ki_l * self.sum_x
                dx = self.kd_l * (err_x - self.prv_err_x) / self.dt
                PID_x = px + sum_x + dx               
                
                self.prv_err_x = err_x
                self.prv_err_a = err_a
                
                PID_x = min(PID_x,0.1)

                if PID_a <= 0:
                    PID_a = max(PID_a, -1.5)
                else:
                    PID_a = min(PID_a, 1.5)
                    
                twist = Twist()
                twist.linear.x = PID_x
                twist.angular.z = PID_a
                self.cmd_publisher.publish(twist)
                
                rate.sleep()
                dist = self.dictance_calc(self.next_x, self.next_y, self.current_x, self.current_y)
                prv_rotation = self.current_angle
         
            
        

            count+=1


    def dictance_calc(self, x1,y1,x2,y2):
        d = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        return float(d)

if __name__ == "__main__":
    controller = Controller()
    
    controller.run()
# if PID_a > 0:
                #     PID_a = min(pa, 1.5)
                # else:
                #     PID_a = max(pa, -1.5)


                # err_a = shortest_angular_distance(self.current_angle, next_angle)
                # if abs(err_a) < 0.04:
                #     err_a = 0

                # prv_rotation = self.current_angle

   # rotation = self.rotation_goal(next_angle)

                


                # err_a = self.shortest_angle_between(next_angle, self.current_angle)
       
                # self.current_x, self.current_y, self.current_angle = self.get_heading()

                # dist = self.dictance_calc(self.next_x, self.next_y, self.current_x, self.current_y)

                
                # err_x = abs(self.next_x - self.current_x)
                                #     self.current_angle = -2*math.pi + self.current_angle

    
    
    # def shortest_angle_between(self, angle1, angle2):
    #     """
    #     Calculates the shortest angle between two angles in degrees.
    #     """
    #     diff = angle2 - angle1
    #     diff = (diff + 180) % 360 - 180
    #     return abs(diff)
    
    # def rotation_goal(self, alpha_rotation):
       
    #     beta_rotation = alpha_rotation + 2 * math.pi
    #     gamma_rotation = alpha_rotation - 2 * math.pi
        
    #     rotations = [abs(alpha_rotation), abs(beta_rotation), abs(gamma_rotation)]
    #     min_indx = rotations.index(min(rotations))

    #     if min_indx == 0:
    #         rotation = alpha_rotation
    #     elif min_indx == 1:
    #         rotation = beta_rotation
    #     elif min_indx == 2:
    #         rotation = gamma_rotation

    #     return rotation

    # def rotate(self):
    #     self.cmd_publisher.publish(Twist())
    #     rospy.sleep(1)

    #     remaining = self.goal_angle
    #     prev_angle = self.get_heading()
        
    #     twist = Twist()
    #     twist.angular.z = self.angular_speed
    #     self.cmd_publisher.publish(twist)
        
    #     # rotation loop
    #     while remaining >= self.epsilon:
    #         current_angle = self.get_heading()
    #         delta = abs(prev_angle - current_angle)
    #         remaining -= delta
    #         prev_angle = current_angle
    
    #     self.cmd_publisher.publish(Twist())

