#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import rospy
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose


class FaceTrackerROS:
    """
    A ROS node that detects faces in webcam images and controls a TurtleSim robot
    to follow the detected face coordinates.
    """

    def __init__(self):
        """Initialize the FaceTrackerROS node with all necessary components."""
        # Face detection setup
        self.face_cascade = cv.CascadeClassifier(
            cv.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )
        if self.face_cascade.empty():
            raise Exception("Error: Could not load face cascade classifier")

        # Face detection parameters
        self.scale_factor = 1.3
        self.min_neighbors = 8
        self.min_size = (30, 30)
        self.box_color = (0, 255, 0)
        self.box_thickness = 2

        # ROS initialization
        rospy.init_node("webcam_subscriber", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/usb_cam_node/image_raw", Image, self.image_callback
        )
        self.cmd_vel_pub = rospy.Publisher(
            '/turtle1/cmd_vel', Twist, queue_size=10
        )
        self.pose_sub = rospy.Subscriber(
            '/turtle1/pose', Pose, self.pose_callback
        )

        # Image processing variables
        self.frame = None
        self.frame_width = None
        self.frame_height = None

        # Turtle state
        self.turtle_pose = None

        # Tracking target
        self.target = None

        # Motion control parameters
        self.linear_speed = 1.0
        self.angular_speed = 1.5
        self.distance_threshold = 0.5

        rospy.loginfo("Face Tracker ROS Node Started")

        # Wait for first frame
        while self.frame is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def detect_faces(self, frame):
        """Detect faces in the given frame using Haar cascade classifier."""
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
            minSize=self.min_size
        )
        return faces

    def draw_boxes(self, frame, faces):
        """Draw rectangles around detected faces and mark their centers."""
        centers = []
        for (x, y, w, h) in faces:
            cv.rectangle(frame, (x, y), (x+w, y+h), self.box_color, self.box_thickness)
            center_x = x + (w // 2)
            center_y = y + (h // 2)
            centers.append((center_x, center_y))
            cv.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            print(f"Face center: ({center_x}, {center_y})")
        return frame, centers

    def map_to_turtlesim(self, center_x, center_y):
        """Map pixel coordinates to TurtleSim coordinates."""
        turtlesim_x = (center_x / self.frame_width) * 11.0
        turtlesim_y = ((self.frame_height - center_y) / self.frame_height) * 11.0
        return turtlesim_x, turtlesim_y

    def control_robot(self, centers):
        """Control the TurtleSim robot based on detected face position."""
        twist = Twist()

        if not centers or self.turtle_pose is None or self.frame_width is None:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        center_x, center_y = centers[0]
        target_x, target_y = self.map_to_turtlesim(center_x, center_y)
        self.target = Point(x=target_x, y=target_y)

        curr_x = self.turtle_pose.x
        curr_y = self.turtle_pose.y
        curr_angle = self.turtle_pose.theta

        dx = target_x - curr_x
        dy = target_y - curr_y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)

        angle_diff = target_angle - curr_angle
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        if distance > self.distance_threshold:
            twist.angular.z = self.angular_speed * angle_diff
            twist.angular.z = max(-self.angular_speed, min(self.angular_speed, twist.angular.z))
            
            if abs(angle_diff) < math.pi/4:
                twist.linear.x = self.linear_speed * distance
                twist.linear.x = max(0.0, min(self.linear_speed, twist.linear.x))
            else:
                twist.linear.x = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            print(f"Target reached: ({target_x:.2f}, {target_y:.2f})")

        self.cmd_vel_pub.publish(twist)

    def pose_callback(self, data):
        """Update turtle pose from incoming pose messages."""
        self.turtle_pose = data

    def image_callback(self, data):
        """Process incoming webcam frames."""
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.frame_width is None:
                self.frame_height, self.frame_width = self.frame.shape[:2]

            faces = self.detect_faces(self.frame)
            frame_with_boxes, centers = self.draw_boxes(self.frame, faces)
            self.control_robot(centers)

            cv.imshow('Face Tracker', frame_with_boxes)
            if cv.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User requested shutdown")

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def run(self):
        """Main execution loop for the ROS node."""
        try:
            rospy.spin()
        finally:
            cv.destroyAllWindows()


if __name__ == "__main__":
    try:
        tracker = FaceTrackerROS()
        tracker.run()
    except Exception as e:
        print(f"An error occurred: {e}")
        cv.destroyAllWindows()
