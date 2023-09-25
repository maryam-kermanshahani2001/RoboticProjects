#!/usr/bin/python3

import tf
import math
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class VFH():

    def __init__(self):

        rospy.init_node('vfh_node', anonymous = False)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        self.a = 1
        self.b = 0.25
        self.l = 2
        self.s_max = 15
        self.thresh = 6
        self.path_length = 0.5
        self.sector_size = 5
        self.goal_index = -1
        self.linear_epsilon = 0.3
        self.ang_epsilon = 0.1
        self.linear_vel = 0.1
        self.ang_vel = 0.1


    def run(self):
        goal_x_arr = [4.5, 3.0, 0.5 , 1.5, 3.5, 6, 7, 8, 13]
        goal_y_arr = [0.5 , 4.5, 2.0 , 5.5, 6.5, 6, 3, 7, 7]
        for i in range(len(goal_x_arr)):
            goal_x = goal_x_arr[i]
            goal_y = goal_y_arr[i]

            while not rospy.is_shutdown():
                vfh_class.guidance(goal_x, goal_y)
                current_position = self.get_pose()[0]
                rospy.loginfo(f"Current goal: ({goal_x}, {goal_y})")

                if (abs(current_position.x - goal_x) < 0.3) and (abs(current_position.y - goal_y) < 0.3):
                    rospy.loginfo(f"Robot reached current goal.")
                    break


    def guidance(self, curr_goal_x, curr_goal_y):

        sectors = self.find_h()
        goal_sector = self.get_goal_sector(curr_goal_x, curr_goal_y)
        selected_sectors = self.calculate_thresh(sectors)

        if sectors[goal_sector] < self.thresh:
            best_sector = goal_sector
            
        else:
            best_sector = self.select_valley(selected_sectors, goal_sector)

        if best_sector > 36:
            best_sector -= 72

        angle = math.radians(best_sector * 5)
        self.vfh_controller(angle)


    def get_pose(self):
        odom = rospy.wait_for_message("/odom", Odometry)
        orientation = odom.pose.pose.orientation
        pose = odom.pose.pose.position
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x, orientation.y, orientation.z, orientation.w
        ))

        return pose, yaw


    def find_h(self):
       h = []
       self.laser_scan = rospy.wait_for_message("/scan", LaserScan)
       self.sector_num = int(len(self.laser_scan.ranges) / self.sector_size)
       
       for i in range(self.sector_num):
           sum_m = 0
           for j in range(i * self.sector_size, (i + 1)*self.sector_size):
               d = min(6, self.laser_scan.ranges[j])
               m = self.a - self.b * d
               sum_m += m
           
           h.append(sum_m)
       
       return self.get_h_prime(h)


    def get_h_prime(self, sectors):
        h_prime = []

        for i in range(self.sector_num):
            total_h = 0
            for j in range(-2,3):
                if abs(j) == 2:
                    k = 1
                    
                else:
                    k = 2
                
                if  self.sector_num <= i + j:
                    j = j * -1
                    i  = self.sector_num - i - 1

                total_h += k * sectors[i+j]

            total_h = total_h / (2*self.l + 1)
            h_prime.append(total_h)
        
        return h_prime


    def calculate_thresh(self ,sectors):

        thresh_arr = []
        for i in range(self.sector_num):
            
            if sectors[i] < self.thresh:
                thresh_arr.append(i)
        
        return thresh_arr


    def get_goal_sector(self, curr_goal_x, curr_goal_y):

        pose, yaw = self.get_pose()
        angle = math.atan2(curr_goal_y - pose.y, curr_goal_x - pose.x)

        if angle < 0:
            angle += 2 * math.pi
        dif = angle - yaw

        if dif < 0:
            dif += 2 * math.pi
        
        goal_idx = int(math.degrees(dif) / self.sector_size) 
        goal_sector_ = goal_idx % self.sector_num
        return goal_sector_
        

    def calculate_valley_arr(self, selected_sectors):
        valley_arr = []
        curr_valley=[]
        for i in range(len(selected_sectors)):
            j = i - 1

            if i == 0 :
                curr_valley.append(selected_sectors[i])
                continue
            
            if selected_sectors[i] - selected_sectors[j] > 1:
                valley_arr.append(curr_valley)
                curr_valley = []

            curr_valley.append(selected_sectors[i])

        valley_arr.append(curr_valley)
        curr_valley = []

        if  valley_arr[-1][-1] == (self.sector_num -1) and valley_arr[0][0] == 0:
            curr_valley = valley_arr.pop(0)
            for i in curr_valley:
                valley_arr[-1].append(i)

        return valley_arr


    def select_valley(self, selected_sectors, goal_sector):
        curr_idx = 0
        curr_min= math.inf
        valley_arr = self.calculate_valley_arr(selected_sectors)
        for i in range(len(valley_arr)):

            for j in range(len(valley_arr[i])):
                
                dist = abs(goal_sector - valley_arr[i][j])
                if dist > 36:
                    dist = 72 - dist

                if dist < curr_min:
                    curr_idx = i
                    curr_min = dist

        nearest_valley = valley_arr[curr_idx]
        
        if len(nearest_valley) <= self.s_max:
            candidate = nearest_valley[int(len(nearest_valley) / 2)]
            return candidate
        
        else :
            candidate = nearest_valley[curr_idx + int(self.s_max / 2)]
            return candidate
        

    def vfh_controller(self, angle):
        prev_angle = self.get_pose()[1]
        remaining = angle

        rospy.sleep(1)
        sign = 1
        if angle > math.pi:
            angle -= 2 * math.pi

        if angle < -math.pi:
            angle += 2 *math.pi

        if angle < 0:
            sign = -1

        twist = Twist()
        twist.angular.z = sign * self.ang_vel
        self.cmd_vel.publish(twist)

        while  self.ang_epsilon <= abs(remaining):

            curr_angle = self.get_pose()[1]
            delta = curr_angle - prev_angle

            if abs(delta) < 0.2:
                remaining -= delta

            prev_angle = curr_angle


        twist.angular.z = 0
        self.cmd_vel.publish(twist)
        remaining = self.path_length
        prev_pose = self.get_pose()[0]

        rospy.sleep(1)

        twist = Twist()
        twist.linear.x = self.linear_vel
        self.cmd_vel.publish(twist)

        while  self.linear_epsilon <= remaining:
            curr_pose = self.get_pose()[0]
            delta = np.linalg.norm([curr_pose.x - prev_pose.x, curr_pose.y - prev_pose.y])
            remaining -= delta
            remaining = abs(remaining)
            prev_pose = curr_pose

        twist.linear.x = 0
        self.cmd_vel.publish(twist)
        rospy.sleep(1)
        self.cmd_vel.publish(Twist())


if __name__ == '__main__':
    try:
        vfh_class = VFH()
        vfh_class.run()

    except rospy.ROSInterruptException:
        pass
        