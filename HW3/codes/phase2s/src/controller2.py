#!/usr/bin/python3
import rospy
import tf
import numpy as np
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, pi, radians,dist
import matplotlib.pyplot as plt

class Controller:
    
    def __init__(self) -> None:
        rospy.init_node("controller" , anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)

        # getting specified parameters
       #self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        #self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        #self.goal_angle = radians(rospy.get_param("/controller/goal_angle")) # rad
        #self.epsilon = rospy.get_param("/controller/epsilon")
        self.twist = Twist()
        
        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO 
# star
        # self.kp_l = 0.2
        # self.ki_l = 0
        # self.kd_l = 0

        # self.kp_angle = 0.5
        # self.ki_angle = 0.0001
        # self.kd_angle = 0

        self.threshold = 0.25
        # star2
# angle
        # self.kp_angle = 2.6
        # self.ki_angle = 0.001
        # self.kd_kd_angle = 0.9

        # #distance
        # self.kp_l = 0.75
        # self.ki_l = 0.001
        # self.kd_l = 0.05
        

        # logarithm
        # self.kp_l = 15
        # self.ki_l = 20
        # self.kd_l = 1

        # self.kp_angle = 1
        # self.ki_angle = 0.03
        # self.kd_angle = 0.05

        # self.threshold = 0.25
        
        # rectangle
        self.kp_l = 0.2
        self.ki_l = 0.02
        self.kd_l = 2

        self.kp_angle = 1
        self.ki_angle = 0.03
        self.kd_angle = 0.05

        self.errs = []
        self.shape = []
        
    
    
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
    
    

    def rotate(self):
        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)

        remaining = self.goal_angle
        _, _, prev_angle = self.get_heading()
        # prev_angle = self.get_heading2()
        
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_publisher.publish(twist)
        
        # rotation loop
        while remaining >= self.epsilon:
            _, _, current_angle = self.get_heading()
            # current_angle = self.get_heading2()
            delta = abs(prev_angle - current_angle)
            remaining -= delta
            prev_angle = current_angle
        
        self.cmd_publisher.publish(Twist())
    

    def make_rectangle(self):
        rectangle = []
        
        X1 = np.linspace(0, 4 , 5)
        for x in X1:
            rectangle.append([x,0.0])

        X3 = np.linspace(4, 0 , 5)
        for x in X3:
            rectangle.append([x,6.0])


        Y2 = np.linspace(0, 4 , 5)
        for y in Y2:
            rectangle.append([0.0,y])

        Y4 = np.linspace(4, 0 , 5)
        for y in Y4:
            rectangle.append([y,6.0])
        self.shape.append(rectangle)


    def make_star(self):
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
        

        self.shape.append(star)


        
    def make_logarithem(self):
        log = []
        a = 0.17
        k = math.tan(a)
        for i in range(150):
            t = i / 20 * math.pi
            dx = a * math.exp(k * t) * math.cos(t)
            dy = a * math.exp(k * t) * math.sin(t)
            log.append([dx,dy])
        self.shape.append(log)

    
    def control(self, goal, prv_yaw):
        previous_distance = 0
        # previous_goal_angle = 0
        sum_distance = 0
        
        current_x, current_y, _ = self.get_heading()
        
        next_y = goal[1]
        next_x = goal[0]
        distance = math.sqrt((next_x-current_x)**2 + (next_y-current_y)**2)
        
        # sum_a = 0
        # self.sum_a = 0
        # self.prv_err_a  = 0
        
        
        while distance > self.threshold:
            current_x, current_y, current_yaw = self.get_heading()
            
            
            next_angle = atan2(next_y-current_y ,  next_x - current_x) 
            
            
            if prv_yaw > pi-0.1 and current_yaw <= 0:
                current_yaw =  2 * pi + current_yaw
            
            elif prv_yaw < -pi+0.1 and current_yaw > 0:
                current_yaw = -2 * pi + current_yaw
            
            if next_angle < -pi/4 or next_angle > pi/4:
                if next_y< 0 and current_y < next_y:
                    next_angle = -2*pi + next_angle
                
                elif next_y >= 0 and current_y > next_y:
                    next_angle = 2*pi + next_angle
            
            distance = math.sqrt((next_x-current_x)**2 + (next_y-current_y)**2)
            diff_distance = distance - previous_distance
          
            angular_error = next_angle - current_yaw
            p_a = self.kp_angle*(angular_error)
            self.twist.angular.z = p_a
            

            # distance = dist([next_x,next_y],[current_x,current_y])
            self.errs.append(distance)
            
            p_l = self.kp_l * distance
            i_l = self.ki_l * sum_distance
            d_l = self.kd_l * diff_distance
            pid_l = p_l+ i_l + d_l


            # err_a = next_angle - current_yaw
            # pa = self.kp_angle * err_a
            # self.sum_a +=  (err_a )
            # sum_a = self.ki_angle * self.sum_a
            # da = self.kd_angle * (err_a - self.prv_err_a) 
            # PID_a = pa + sum_a + da            
           
           

            self.twist.linear.x = min(pid_l,0.1)
            
            # self.twist.angular.z = PID_a


            
            if self.twist.angular.z <= 0:
                self.twist.angular.z = max(self.twist.angular.z, -1.5)
            else:
                self.twist.angular.z = min(self.twist.angular.z, 1.5)
            
            self.cmd_publisher.publish(self.twist)

            rospy.sleep(1)
            previous_distance = distance
            sum_distance = sum_distance + distance
            prv_yaw = current_yaw

            # self.prv_err_a = err_a

            # sum_angle = sum_angle + current_yaw
    
    
    def on_shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_publisher.publish(Twist())
        plt.plot(list(range(len(self.errs))),
                    self.errs, label='errs')
        plt.axhline(y=0,color='R')
        plt.draw()
        plt.legend(loc="upper left", frameon=False)
        plt.savefig(f"errs_{self.kp_l}_{self.kd_l}_{self.ki_l}.png")
        plt.show()

        rospy.sleep(1)
            

   


    def run(self):
        # self.make_logarithem()
        # self.make_star()
        
        self.make_rectangle()
        

        while not rospy.is_shutdown():
            prv_yaw = 0
            for i,goal in enumerate(self.shape[0]):
                self.cmd_publisher.publish(Twist())                
                self.control(goal, prv_yaw)

                

                
if __name__ == "__main__":
    controller = Controller()
    controller.run()
