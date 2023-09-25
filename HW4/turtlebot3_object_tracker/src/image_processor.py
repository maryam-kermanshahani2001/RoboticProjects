#!/usr/bin/python3

# Python
import copy

# Object detection
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator
from ultralytics.yolo.engine.results import Results
from turtlebot3_object_tracker.srv import Detection, DetectionResponse
# ROS
import rospy
from sensor_msgs.msg import Image


class ImageProcessor:
    def __init__(self) -> None:
        # Image message
        self.image_msg = Image()

        self.image_res = 240, 320, 3 # Camera resolution: height, width
        self.image_np = np.zeros(self.image_res) # The numpy array to pour the image data into

        # TODO: Subscribe on your robot's camera topic
        #me: image_suscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_image)

        # NOTE: Make sure you use the provided listener for this subscription
        self.camera_subscriber  = rospy.Subscriber('/follower/camera/image', Image, self.camera_listener) #me
        
        # TODO: Instantiate your YOLO object detector/classifier model
        self.model: YOLO = YOLO()  #me
        # TODO: You need to update results each time you call your model
        # self.results: Results =  self.model.predict(self.image_np) #me: 

        self.cv2_frame_size = 400, 320
        cv2.namedWindow("robot_view", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robot_view", *self.cv2_frame_size)

        # TODO: Setup your "human detection" service        
        self.human_detection_server = rospy.Service('detection', Detection, self.serv_callback)

        self.cords = None
        self.update_view()


    def serv_callback(self, req):
        self.label = req.label
        self.results = self.model(self.image_np)
        resp = self.calculate_bb()
        return resp


    def camera_listener(self, msg: Image):
        self.image_msg.data = copy.deepcopy(msg.data)

    # def callback_image(self, msg:Image):
        # pass

   

    def update_view(self):
        try:
            while not rospy.is_shutdown():
                if len(self.image_msg.data) == 0: # If there is no image data
                    continue

                # Convert binary image data to numpy array
                self.image_np = np.frombuffer(self.image_msg.data, dtype=np.uint8)
                self.image_np = self.image_np.reshape(self.image_res)

                frame = copy.deepcopy(self.image_np)

                
                # TODO: You can use an "Annotator" to draw object bounding boxes on frame
                annotator = Annotator(frame , line_width=1, font_size=1)
                if self.cords:
                    annotator.box_label(box= self.cords, color=(255,0,0), txt_color=(255,255,255))
                
                
                
                # cv2.imshow("robot_view", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                cv2.imshow("robot_view",cv2.cvtColor(annotator.result(), cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)

        except rospy.exceptions.ROSInterruptException:
            pass


    def calculate_bb(self):
        #me     [[x1,y1,x2,y2,object_type,probability],..]
        result = self.results[0]
        # box = result.boxes[0]
        
        # output = {}
        
        # output = []                    

        detected = False
        self.cords = None
        
        for res in self.results:
            if len(res.boxes.xyxy) ==0 or res.names is None:
                continue
            if self.label == res.names[0]:
                detected = True
                box = res.boxes
        # for box in result.boxes:
                self.cords = box.xyxy[0].tolist()
                # x1, y1, x2, y2 = [round(x) for x in cords]
                # bb_center_x = (x1 + x2) / 2
                # bb_center_y = (y1 + y2) / 2
                # bb_width = x2 - x1
                # bb_height = y2 - y1

        # x1, y1, x2, y2 = [
        # round(x) for x in box.xyxy[0].tolist()
        # ]
                # class_id = box.cls[0].item()
                # prob = round(box.conf[0].item(), 2)
                # output[result.names[class_id]] = [bb_center_x, bb_center_y, bb_width, bb_height, prob ]
                
            # output.append([
            # bb_center_x, bb_center_y, bb_width, bb_height, result.names[class_id], prob
            # ])
        resp = DetectionResponse()

        if detected and self.cords is not None:
            resp.detected = detected

            x1, y1, x2, y2 = [round(x) for x in self.cords]
            

            resp.x_bb = (x1 + x2) / 2
            resp.y_bb = (y1 + y2) / 2
            resp.width_bb = x2-x1
            resp.height_bb = y2-y1
            # resp.x_bb = output[self.label][0]
            # resp.y_bb = output[self.label][1]
            # resp.width_bb = output[self.label][2]
            # resp.height_bb = output[self.label][3]
            resp.width_img = self.image_res[0]
            resp.height_img = self.image_res[1]

        return resp
        # max_probability = 0.0
        # box_with_max_probability = None
        # for box in result.boxes:
        #     cords = box.xyxy[0].tolist()
        #     x1, y1, x2, y2 = [round(x) for x in cords]
        #     bb_center_x = (x1 + x2) / 2
        #     bb_center_y = (y1 + y2) / 2
        #     bb_width = x2 - x1
        #     bb_height = y2 - y1

        #     class_id = box.cls[0].item()
        #     prob = round(box.conf[0].item(), 2)

        #     if prob > max_probability:
        #         max_probability = prob
        #         box_with_max_probability = [bb_center_x, bb_center_y, bb_width, bb_height, result.names[class_id], prob]
        # box_with_max_probability

        


if __name__ == "__main__":
    rospy.init_node("image_processor", anonymous=True)

    rospy.on_shutdown(cv2.destroyAllWindows)

    image_processor = ImageProcessor()

    rospy.spin()


