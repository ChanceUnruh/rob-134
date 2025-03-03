import cv2 as cv
import numpy as np


class SelectColor:

    def __init__(self):
        self.max_value = 255
        self.max_value_h = 180
        self.low_H = 0
        self.low_S = 0
        self.low_V = 0
        self.high_H = self.max_value_h
        self.high_S = self.max_value
        self.high_V = self.max_value
        self.window_capture_name = 'Video Capture'
        self.window_detection_name = 'HSV Values'
        self.cap = cv.VideoCapture(0)
    
    def low_H_trackbar(self, val):
        self.low_H = min(self.high_H - 1, val)
        cv.setTrackbarPos("Low H", self.window_detection_name, self.low_H)

    def high_H_trackbar(self, val):
        self.high_H = max(val, self.low_H + 1)
        cv.setTrackbarPos("High H", self.window_detection_name, self.high_H)

    def low_S_trackbar(self, val):
        self.low_S = min(self.high_S - 1, val)
        cv.setTrackbarPos("Low S", self.window_detection_name, self.low_S)

    def high_S_trackbar(self, val):
        self.high_S = max(val, self.low_S + 1)
        cv.setTrackbarPos("High S", self.window_detection_name, self.high_S)

    def low_V_trackbar(self, val):
        self.low_V = min(self.high_V - 1, val)
        cv.setTrackbarPos("Low V", self.window_detection_name, self.low_V)

    def high_V_trackbar(self, val):
        self.high_V = max(val, self.low_V + 1)
        cv.setTrackbarPos("High V", self.window_detection_name, self.high_V)

    def SetupWindows(self):
        # Create OpenCV windows
        cv.namedWindow(self.window_capture_name)
        cv.namedWindow(self.window_detection_name)

        # Create trackbars for HSV adjustment
        cv.createTrackbar("Low H", self.window_detection_name, self.low_H, self.max_value_h, self.low_H_trackbar)
        cv.createTrackbar("High H", self.window_detection_name, self.high_H, self.max_value_h, self.high_H_trackbar)
        cv.createTrackbar("Low S", self.window_detection_name, self.low_S, self.max_value, self.low_S_trackbar)
        cv.createTrackbar("High S", self.window_detection_name, self.high_S, self.max_value, self.high_S_trackbar)
        cv.createTrackbar("Low V", self.window_detection_name, self.low_V, self.max_value, self.low_V_trackbar)
        cv.createTrackbar("High V", self.window_detection_name, self.high_V, self.max_value, self.high_V_trackbar)

    def SelectColorRun(self):
        if not self.cap.isOpened():
            print("Error: Could not open video capture")
            return None
            
        print("Press 's' to save HSV values, 'q' to quit")
        self.SetupWindows()

        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            # Convert frame from BGR to HSV color space
            frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            
            # Apply thresholding to isolate selected color range
            frame_threshold = cv.inRange(frame_HSV, 
                                       (self.low_H, self.low_S, self.low_V), 
                                       (self.high_H, self.high_S, self.high_V))

            # Display original and thresholded images
            cv.imshow(self.window_capture_name, frame)
            cv.imshow(self.window_detection_name, frame_threshold)
            
            # Wait for key press
            key = cv.waitKey(30) & 0xFF
            
            if key == ord('s') or cv.getWindowProperty(self.window_capture_name, cv.WND_PROP_VISIBLE) < 1:  # Save values and exit
                print(f"Saved HSV Values: Low H: {self.low_H}, Low S: {self.low_S}, Low V: {self.low_V}, "
                      f"High H: {self.high_H}, High S: {self.high_S}, High V: {self.high_V}")
                self.cleanup()
                return (self.low_H, self.low_S, self.low_V, 
                        self.high_H, self.high_S, self.high_V)
            
            elif key == ord('q') or cv.getWindowProperty(self.window_detection_name, cv.WND_PROP_VISIBLE) < 1:
                break

        self.cleanup()
        return None

    def cleanup(self):
        self.cap.release()
        cv.destroyAllWindows()


# if __name__ == "__main__":
#     selector = SelectColor()
#     hsv_values = selector.run()
#     if hsv_values:
#         print(f"Returned HSV values: {hsv_values}")