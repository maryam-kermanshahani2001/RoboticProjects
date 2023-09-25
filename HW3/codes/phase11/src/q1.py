#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import tf
import math

class PIDController():


    def __init__(self, k_p, k_i, k_d):
        
        rospy.init_node('wall_follower', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        
        self.k_i = k_i
        self.k_p = k_p
        self.k_d = k_d
        
        self.dt = 0.05
        self.v = 0.6
        self.D = 10
        rate = 1/self.dt
        
        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.errs = []

    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        _, _, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        return x, y, yaw
    

    def distance_from_dest(self):
        x, y, _ = self.get_heading()
        # d = abs(10 - x)
        d = math.sqrt((10 - x) ** 2 + (0 - y) ** 2)
        print(d)
        return float(d)


    def follow_dest(self):
        
        d = self.distance_from_dest()    
        sum_i = 0
        prev_error = 0
        
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.v
        err = self.distance_from_dest()


        while not rospy.is_shutdown() and err>=0.4:
            self.cmd_vel.publish(move_cmd)

            err = self.distance_from_dest()
            self.errs.append(err)
            sum_i += err * self.dt
            
            P = self.k_p * err
            I = self.k_i * sum_i
            D = self.k_d * (err - prev_error)/self.dt
            prev_error = err
            move_cmd.linear.x = P + I + D       
        
            rospy.loginfo("*******************")
            rospy.loginfo(f'k_p {self.k_p}')
        
            rospy.loginfo(f'k_i {self.k_i}')
            rospy.loginfo(f'k_d {self.k_d}')
            rospy.loginfo("*******************")



            rospy.loginfo(f"P : {P} I : {I} D : {D}")
            # move_cmd.angular.z = P + I + D 
            
            rospy.loginfo(f"error : {err} speed : {move_cmd.linear.x} theta : {move_cmd.angular.z}")
            
            d = self.distance_from_dest()

            self.r.sleep()

    def on_shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        plt.plot(list(range(len(self.errs))),
                    self.errs, label='errs')
        plt.axhline(y=0,color='R')
        plt.draw()
        plt.legend(loc="upper left", frameon=False)
        plt.savefig(f"errs_{self.k_p}_{self.k_d}_{self.k_i}.png")
        plt.show()

        rospy.sleep(1)
            

if __name__ == '__main__':
    
    # flag = True
    # a = input("Enter Mode: 1 for p, 2 for pd, 3 for pid, 0 for exit")
    # if a == 0:
        # flag = False
    # else:
    # err = 0.4
    a = 3
        
    if a == 1:
        k_i = 0.0
        k_p = 0.015
        k_d = 0

    elif a == 2:
        k_i = 0.0
        k_p = 0.02
        k_d = 0.01
    
    elif a == 3:
        k_i = 0.001
        k_p = 0.02
        k_d = 0.01
        
    try:
        pidc = PIDController(k_p, k_i, k_d)
        pidc.follow_dest()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")


