#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO


class ImageSubscriberNode(Node):

    package_share = get_package_share_directory('weed_vision')
    model = YOLO(package_share + '/models/yolo.pt')

    crop_color = (255, 0,   0,   128)  # Blue  color with 50%  transparency (BGR with alpha)
    weed_color = (0,   0,   255, 128)  # Red   color with 50%  transparency (BGR with alpha)
    text_color = (255, 255, 255, 255)  # Black color with 100% transparency (BGR with alpha)

    id2color = {0: crop_color, 1: weed_color}

    def __init__(self):
        super().__init__('image_subscriber_node')
        self.subscription1 = self.create_subscription(
            Image,
            '/front_depth_camera/image_raw',  # Replace with your first image topic
            self.image1_callback,
            10)
        self.subscription1  # Prevent unused variable warning
        self.subscription2 = self.create_subscription(
            Image,
            '/rear_depth_camera/image_raw',  # Replace with your second image topic
            self.image2_callback,
            10)
        self.subscription2  # Prevent unused variable warning
        self.cv_bridge = CvBridge()

    def image1_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.inference(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error converting or displaying image from topic 1: {str(e)}')

    def image2_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.inference(cv_image, window_name="rear")
        except Exception as e:
            self.get_logger().error(f'Error converting or displaying image from topic 2: {str(e)}')

    def inference(self, image, window_name="front"):
        result = self.model(image)
        try:
            if not len(result[0].masks) == 0:

                image = result[0].orig_img
                image = cv2.resize(image, (1024, 1024))
                height, width, _ = image.shape
                print(height, width)

                mask = np.zeros_like(image)

                id2class = result[0].names

                for i in range(len(result[0].masks)):

                    # print(id2class[result[0].boxes[i].cls.item()])

                    pixel_polygons = np.array([(int(polygon[0] * width), int(polygon[1] * height)) for polygon in result[0].masks[i].xyn[0]], dtype=np.int32)
                    # box = np.array([(int(box[0] * width), int(box[1] * height)) for box in result[0].boxes[i].xywhn[0]], dtype=np.int32)
                    box = result[0].boxes[i].xyxyn[0]

                    x1 = int(box[0] * height)
                    y1 = int(box[1] * width)

                    cv2.rectangle(
                        image, 
                        (x1, y1), 
                        (int(box[2] * height), int(box[3] * width)), 
                        self.id2color[result[0].boxes[i].cls.item()], 
                        2)

                    cv2.putText(image, id2class[result[0].boxes[i].cls.item()], (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.text_color, 2)

                    try:
                        cv2.fillPoly(mask, [pixel_polygons], self.id2color[result[0].boxes[i].cls.item()])
                    except:
                        pass

                result_image = cv2.addWeighted(image, 1, mask, 1, 0)  # Adjust the alpha value (0.5) for transparency

                # Display the result
                cv2.imshow(window_name, result_image)
                cv2.waitKey(1)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
