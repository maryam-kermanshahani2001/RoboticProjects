#!/usr/bin/python3

# ROS
import rospy
from geometry_msgs.msg import Twist
from turtlebot3_object_tracker.srv import DetectionRequest, Detection
import math


class Controller:
    def __init__(self) -> None:
        # Use these Twists to control your robot
        self.move = Twist()
        self.move.linear.x = 0.1
        self.freeze = Twist()

        # The "p" parameter for your p-controller, TODO: you need to tune this
        self.angular_vel_coef = 1

        # TODO: Create a service proxy for your human detection service
        rospy.wait_for_service('detection')

        self.detection_srv_prx = rospy.ServiceProxy('detection', Detection)

        
        # TODO: Create a publisher for your robot "cmd_vel"
        self.cmd_vel_pub = rospy.Publisher('/follower/cmd_vel', Twist, queue_size=10)


    
    def run(self) -> None:
        try:
            while not rospy.is_shutdown():
                # TODO: Call your service, ride your robot
                req = DetectionRequest()
                req.label = "person"
                resp = self.detection_srv_prx(req)
                if resp.detected: 
                    self.move_robot(resp)
                else:
                    self.cmd_vel_pub.publish(self.freeze)
                    rospy.loginfo("robot freezed")

                rospy.sleep(0.)

        except rospy.exceptions.ROSInterruptException:
            pass

    def move_robot(self, response):
        x_bb, y_bb, width_bb, height_bb, width_img, height_img = response.x_bb, response.y_bb, response.width_bb, response.height_bb, response.width_img, response.height_img
        goal_w = width_img / 2
        goal_h = height_img / 2
        err = goal_w - x_bb
        angle = math.atan2(err, width_img)
        self.move.angular.z = self.angular_vel_coef * angle
        self.cmd_vel_pub.publish(self.move)
        pass

   


                
    
    

if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    
    controller = Controller()
    controller.run()
    

