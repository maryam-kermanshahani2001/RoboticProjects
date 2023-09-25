#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from hw0.msg import proximity
from random import randint, choice, seed
from time import time

def randNumber():
    return randint(10, 200)

def talker():
    ## for String you need import std_msgs.msg...
    ## queue size: y bufferi k age shabke kond bd ya dastorat nmirft to buffer negah midare k dastora az beyn naran
    pub = rospy.Publisher("distance",proximity , queue_size=10)

    # anonymous: farz kn chand node to har kodum initialize kardn, age ham nam bashe overwrite mishe
    # in y meqdar randomi b tahesh ezafe mikne ta motmaen bashe in node faqt y done bashe.
    rospy.init_node("sensor", anonymous=True) 

    # y chize mohem dg ine k chandvqt y bar loop  ejra beshe
    # input adad ferekans masln 10 yani dar har 1s 10 bar navasan kone(HZ)
    rate = rospy.Rate(1)
    i = 0

    
    # y loop bayad drst knim k shart khoroj interrupt b terminale.
    while not rospy.is_shutdown():
        # karo dakhel loop anjam midim initialize ro qable loop
        msg = proximity()
        msg.up = randNumber()
        msg.down = randNumber()
        msg.right = randNumber()
        msg.left = randNumber()
        
        # log hamun print
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

        # for testing
        # i += 1
        # if i == 10:
            # break


if __name__ == "__main__":
    talker()

# bade taqirat bayad dar pohse catkin_ws y bar dg catkin_make ro anjam bedim
# baraye run kardn haal roscore ro miznim
# bad terminal jadid baz miknim o miarim dar catkin_ws
# baraye run kardn node rosrun miznim vali ...
# hw0 nmishnase dalilesh ine setup.bash devel ro src nkardm => . devel/setup.bash
# hala rosrun hw0 setup.bash ro miznm


# baraye msg custom folder hw0/msg
# fili ijad miknm dakhelesh type o esm ro mizarm
# wikiros tutorials creating msg website (http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
# execute dep yani vaqti node exec mishe b che depend hayi niaz dre
# bad CMakeLists.txt in hw0
# bade hameye taqirate site yadet bashe dakhel include_directories include dovom ro ham uncomment kni 
# dobare vaqti tuye catkin_ws hasti catkin_make ro seda mikni
# y baram vscode ro mibandi dobare baz mikni
# bad ketabkhone import mishe
# badeshm hatmn bayad .devel/setup.bash ro dobare bzni
# rosmsg show hw0/proximity
# rosrun hw0 sensor.py
# 