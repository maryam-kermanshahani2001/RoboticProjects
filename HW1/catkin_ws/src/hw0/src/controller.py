#!/usr/bin/python3

import rospy
from hw0.msg import motor_topic, proximity


def callback(msg: proximity):
    
    rotation = 0
    clockwise = True

    motopub = rospy.Publisher("motor_rotate",motor_topic , queue_size=10)

    # create msg
    moto_msg = motor_topic()

    # solution duo to question 
    minimum_distance = min(msg.up, msg.down, msg.left, msg.right)
    if minimum_distance != msg.up:
        if minimum_distance != msg.down:
            if minimum_distance == msg.right:
                rotation = 90
                clockwise = True

            else:
                rotation = 90
                clockwise = False
                

        else:
            rotation = 180
            clockwise  = True # no difference
            

    else:
        rotation = 0
    
  
    moto_msg.rotation = rotation
    moto_msg.clockwise = clockwise

   
    rospy.loginfo('Publisher - Motor1 and Motor2 message {}'.format(moto_msg))
    motopub.publish(moto_msg)


def listener():
    rospy.init_node('Controller', anonymous=True)
    rospy.Subscriber("distance", proximity, callback=callback)
    rospy.spin()
    

if __name__ == "__main__":
    listener()
    