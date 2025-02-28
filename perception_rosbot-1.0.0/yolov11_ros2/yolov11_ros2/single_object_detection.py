#! /home/asimovo/assets/.venv/bin/python3

import rclpy
from rclpy.node import Node
# image data and processing
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# vision processing
import cv2
import torch
from pathlib import Path
from ultralytics import YOLO
import numpy as np
# message
from std_msgs.msg import String

class ShowingImage(Node):

    def __init__(self):
        super().__init__('my_camera_node')
        # publishers
        self.image_pub = self.create_publisher(Image, "/my_image_output", 10)
        self.object_pub = self.create_publisher(String, "/my_object_detected", 10)
        self.pub_msg = String()
        # bridge for image processing
        self.bridge_object = CvBridge()
        # subscribers
        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.camera_callback, 10)
        
        # Check if CUDA (GPU support) is available
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f'Using device: {self.device}')
        # Model
        self.model = YOLO("yolo11s.pt")
        
        # atributtes for the object detection
        self.object_found = False
        self.list_objects_detected = []
        
    def camera_callback(self, msg):
        try:
            self.object_found = False # clear the flag
            cv_image = self.bridge_object.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            original_img = cv_image.copy()
            
            # Inference and results
            results = self.model(cv_image)[0] # get 1st result
            self.list_objects_detected = [] # clear the list
            # Access detections 
            for result in results.boxes:
                # Get box coordinates, confidence and class
                x1, y1, x2, y2 = result.xyxy[0].cpu().numpy()  # get box coordinates
                confidence = float(result.conf[0].cpu().numpy())  # get confidence 
                class_id = int(result.cls[0].cpu().numpy())    # get class id
                
                # Draw bounding box on the image
                cv2.rectangle(original_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(original_img, f'{self.model.names[class_id]}: {confidence:.2f}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 0), 1)
                
                # Store in list format similar to previous version
                self.list_objects_detected.append({
                    'bbox': [x1, y1, x2, y2],
                    'confidence': confidence,
                    'class_id': class_id,
                    'class_name': self.model.names[class_id]
                })
                
                self.object_found = True                
                            
            cv2.putText(original_img, f'Image Size: {original_img.shape[1]}x{original_img.shape[0]}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            # Display the image with bounding boxes
            cv2.imshow('Detected Objects', original_img)
            # publish the first object detected
            if self.object_found:
                self.pub_msg.data = self.list_objects_detected[0]['class_name']
                self.object_pub.publish(self.pub_msg)
            else:
                self.pub_msg.data = 'nothing'
                self.object_pub.publish(self.pub_msg)
            #publishing a modify image
            self.image_pub.publish(self.bridge_object.cv2_to_imgmsg(original_img, encoding="bgr8"))
            
        except CvBridgeError as e:
            self.get_logger().info('{}'.format(e))
            self.object_found = False

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    showing_image_object = ShowingImage()
    rclpy.spin(showing_image_object)
    showing_image_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# export PYTHONPATH=$PYTHONPATH:/home/user/ros2_ws/src/robot_ur3e_perception/venv/lib/python3.10/site-packages/