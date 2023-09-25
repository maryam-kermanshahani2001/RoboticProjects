#!/usr/bin/python3

import rospy
from distance_calculator.srv import GetNextDestination, GetNextDestinationResponse
import random
from math import sqrt

class DistanceCalculator():
    def __init__(self) -> None:
        pass
    
    def get_distance(self, req):
        try:
            current_x = req.current_x
            current_y = req.current_y
            x = round(random.uniform(-10,10), 2)
            y = round(random.uniform(-10,10), 2)

            flag = 1
            while flag!= 0:
                if dictance_calc(current_x, current_y, x, y) >= 5:
                    flag = 0
                    break
                x = round(random.uniform(-10,10), 2)
                y = round(random.uniform(-10,10), 2)
                
            response = GetNextDestinationResponse()
            response.next_x = x
            response.next_y = y
            return response
            
        except:
            return None


def dictance_calc(x1,y1,x2,y2):
    d = sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    return float(d)


def listener():
    rospy.init_node('mission', anonymous=True)
    dc = DistanceCalculator()
    s = rospy.Service('/get_next_destination', GetNextDestination, dc.get_distance)
    rospy.spin()

if __name__== '__main__':
    listener()