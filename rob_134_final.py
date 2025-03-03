#!/usr/bin/env python3

import cv2 as cv 
import numpy as np 
from sensor_msgs.msg import Image  # ROS message type for camera images
from cv_bridge import CvBridge, CvBridgeError  # Converts between ROS and OpenCV image formats
from geometry_msgs.msg import Twist, Point  # ROS message types for velocity commands and points
from turtlesim.msg import Pose  # ROS message type for turtle position/orientation
import rospy  # ROS Python client library
import math


class FaceTrackerROS:


####################################################################################################
    def __init__(self):
        # Load the pre-trained face detection classifier from OpenCV's data directory
        self.face_cascade = cv.CascadeClassifier(cv.data.haarcascades + 'haarcascade_frontalface_default.xml')
        if self.face_cascade.empty():
            raise Exception("Error: Could not load face cascade classifier")
            
        # Face detection parameters
        self.scale_factor = 1.3  # How much the image size is reduced at each image scale
        self.min_neighbors = 8   # How many neighbors each candidate rectangle should have
        self.min_size = (30, 30) # Minimum face size to detect (width, height) in pixels
        self.box_color = (0, 255, 0)  # Green color in BGR format for drawing rectangles
        self.box_thickness = 2    # Thickness of bounding box lines
        
        # Initialize ROS components
        rospy.init_node("webcam_subscriber", anonymous=True)  # Create ROS node
        self.bridge = CvBridge()  # Initialize CV bridge for image conversion
        self.image_sub = rospy.Subscriber("/usb_cam_node/image_raw", Image, self.image_callback) # Subscribe to webcam image topic
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) # Publisher for turtle velocity commands
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback) # Subscriber for turtle pose updates
        
        # Image processing variables
        self.frame = None         # Current video frame
        self.frame_width = None   # Frame width in pixels
        self.frame_height = None  # Frame height in pixels
        
        # Turtle state
        self.turtle_pose = None   # Current pose (x, y, theta) of turtle
        
        # Tracking target
        self.target = None        # Target position in TurtleSim coordinates
        
        # Motion control parameters
        self.linear_speed = 1.0    # Base linear speed
        self.angular_speed = 1.5   # Base angular speed
        self.distance_threshold = 0.5  # Distance to stop moving toward target
        
        rospy.loginfo("Face Tracker ROS Node Started")
        
        # Wait for first frame to arrive
        while self.frame is None and not rospy.is_shutdown():
            rospy.sleep(0.1)  # Delay
####################################################################################################


    def detect_faces(self, frame):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)  # Convert to grayscale

        # Detect faces with specified parameters
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=self.scale_factor, minNeighbors=self.min_neighbors, minSize=self.min_size)
        return faces 
    

    def draw_boxes(self, frame, faces):
        centers = []  # List to store center points of detected faces
        for (x, y, w, h) in faces:
            # Draw rectangle around face
            cv.rectangle(frame, (x, y), (x+w, y+h), self.box_color, self.box_thickness)
            # Calculate center point
            center_x = x + (w // 2)
            center_y = y + (h // 2)
            centers.append((center_x, center_y))
            # Draw red dot at center
            cv.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            print(f"Face center: ({center_x}, {center_y})")
        return frame, centers
    

    def map_to_turtlesim(self, center_x, center_y):
        # Scale x from frame width (e.g., 640) to 11
        turtlesim_x = (center_x / self.frame_width) * 11.0
        # Scale and invert y (camera y increases downward, TurtleSim upward)
        turtlesim_y = ((self.frame_height - center_y) / self.frame_height) * 11.0
        return turtlesim_x, turtlesim_y
    

    def ControlRobot(self, centers):
        twist = Twist()  # Velocity command message
        
        # If no faces detected or pose unknown, stop
        if not centers or self.turtle_pose is None or self.frame_width is None:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return
        
        # Use first detected face as target
        center_x, center_y = centers[0]
        target_x, target_y = self.map_to_turtlesim(center_x, center_y)
        self.target = Point(x=target_x, y=target_y)
        
        # Get current turtle position and orientation
        curr_x = self.turtle_pose.x
        curr_y = self.turtle_pose.y
        curr_angle = self.turtle_pose.theta
        
        # Calculate vector to target
        dx = target_x - curr_x
        dy = target_y - curr_y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference and normalize to [-pi, pi]
        angle_diff = target_angle - curr_angle
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        
        # Control logic
        if distance > self.distance_threshold:
            # Proportional angular control
            twist.angular.z = self.angular_speed * angle_diff
            # Clamp angular velocity to max speed
            twist.angular.z = max(-self.angular_speed, min(self.angular_speed, twist.angular.z))
            
            # Linear control only when roughly aligned
            if abs(angle_diff) < math.pi/4:  # Within 45 degrees
                twist.linear.x = self.linear_speed * distance
                twist.linear.x = max(0.0, min(self.linear_speed, twist.linear.x))
            else:
                twist.linear.x = 0.0
        else:
            # Target reached, stop moving
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            print(f"Target reached: ({target_x:.2f}, {target_y:.2f})")
        
        self.cmd_vel_pub.publish(twist)


    def pose_callback(self, data):
        # Update turtle pose when new pose message arrives.
        self.turtle_pose = data


    def image_callback(self, data):
        # Process incoming webcam frames.
        try:
            # Convert ROS image message to OpenCV format
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.frame_width is None:
                self.frame_height, self.frame_width = self.frame.shape[:2]
            
            # Detect faces and draw boxes
            faces = self.detect_faces(self.frame)
            frame_with_boxes, centers = self.draw_boxes(self.frame, faces)
            self.ControlRobot(centers)
            
            # Display processed frame
            cv.imshow('Face Tracker', frame_with_boxes)
            if cv.waitKey(1) & 0xFF == ord('q'):  # Quit on 'q' key
                rospy.signal_shutdown("User requested shutdown")
                
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            

    def run(self):
        # Main loop to keep node running.
        try:
            rospy.spin()  # Keep node alive until shutdown
        finally:
            cv.destroyAllWindows()  # Clean up OpenCV windows


if __name__ == "__main__":
    try:
        tracker = FaceTrackerROS()  # Create tracker instance
        tracker.run()              # Start the node
    except Exception as e:
        print(f"An error occurred: {e}")
        cv.destroyAllWindows()  # Ensure cleanup on error