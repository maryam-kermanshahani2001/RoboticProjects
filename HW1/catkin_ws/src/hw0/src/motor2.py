#!/usr/bin/python3

import rospy
from hw0.msg import motor_topic

def callback(msg:motor_topic):
    rospy.loginfo('Motor2: rotation => {} clockwise => {}'.format(msg.rotation, msg.clockwise))



def listener():
    rospy.init_node('Motor2', anonymous=True)
    # rospy.Subscriber("motor2_rotate", motor_topic, callback=callback)
    rospy.Subscriber("motor_rotate", motor_topic, callback=callback)

    rospy.spin()


if __name__=="__main__":
    listener()

