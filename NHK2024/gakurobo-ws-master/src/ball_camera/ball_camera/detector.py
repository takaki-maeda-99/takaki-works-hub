import os
from enum import Enum

import cv2
import cv_bridge
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from ultralytics import YOLO

from ball_camera_msgs.msg import Ball, Balls


class Color(Enum):
    BLUE = 0
    RED = 1
    PURPLE = 2
    UNKNOWN = 3


class Detector(Node):
    def __init__(self):
        super().__init__("detector")

        self._interval_ms = self.declare_parameter("interval_ms", 100).value
        self._model_name = self.declare_parameter("model_name", "yolov8m.pt").value
        self._trim_x_min = self.declare_parameter("trim_x_min", 0).value
        self._trim_x_max = self.declare_parameter("trim_x_max", 0).value
        self._trim_y_min = self.declare_parameter("trim_y_min", 0).value
        self._trim_y_max = self.declare_parameter("trim_y_max", 0).value
        self._purple_rb_threshold = self.declare_parameter(
            "purple_rb_threshold", 40
        ).value
        self._show_frame = self.declare_parameter("show_frame", False).value
        self._center_x_min = self.declare_parameter("center_x_min", 150).value
        self._center_x_max = self.declare_parameter("center_x_max", 250).value
        self._center_y_min = self.declare_parameter("center_y_min", 35).value
        self._center_y_max = self.declare_parameter("center_y_max", 135).value

        self._latest_frame = None
        self._camera_condition_ = False

        self._timer = self.create_timer(self._interval_ms / 1000, self._detect)
        self._pub = self.create_publisher(Balls, "/ball_camera", 10)
        self._sub = self.create_subscription(
            Image, "/image_raw", self._image_callback, 10
        )
        self._camera_cond_sub_ = self.create_subscription(
            Bool, "/camera_condition", self._camera_cond_callback, 10
        )
        self._bridge = cv_bridge.CvBridge()

    def _image_callback(self, img_msg):
        self._latest_frame = self._bridge.imgmsg_to_cv2(
            img_msg, desired_encoding="bgr8"
        )

    def _camera_cond_callback(self, msg):
        self._camera_condition_ = msg.data

    def _detect(self):
        if not self._camera_condition_:
            return

        frame = self._latest_frame
        if frame is None:
            return

        if self._trim_y_min == self._trim_y_max:
            self._trim_y_min, self._trim_y_max = 0, frame.shape[0]
        if self._trim_x_min == self._trim_x_max:
            self._trim_x_min, self._trim_x_max = 0, frame.shape[1]
        frame = frame[
            self._trim_y_min : self._trim_y_max, self._trim_x_min : self._trim_x_max
        ]
        bgr = frame[
            (self._trim_y_max - self._trim_y_min) // 2,
            (self._trim_x_max - self._trim_x_min) // 2,
        ]
        color = self._detect_color(bgr)

        balls = Balls()
        ball = Ball()
        if all([x <= 100 for x in bgr]):
            ball.color = 0
            ball.confidence = float(0)
            ball.center_x = 0
            ball.center_y = 0
        else:
            ball.color = color.value
            ball.confidence = float(1)
            ball.center_x = 0
            ball.center_y = 0
        self.get_logger().info(f"Detected {color.name} ({bgr.tolist()})")
        balls.balls.append(ball)

        self._pub.publish(balls)

        if self._show_frame:
            cv2.waitKey(1)
            cv2.imshow("Ball Detection", frame)

    def _detect_color(self, bgr):
        max_elem = bgr.argmax()
        if abs(int(bgr[0]) - int(bgr[2])) < self._purple_rb_threshold:
            return Color.PURPLE
        elif max_elem == 0:
            return Color.BLUE
        elif max_elem == 1:
            return Color.UNKNOWN
        elif max_elem == 2:
            return Color.RED

    def __del__(self):
        if self._show_frame:
            cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
