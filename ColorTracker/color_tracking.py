import cv2 as cv
import numpy as np
from collections import deque
from hsv_thresholding import SelectColor

class ColorTracker:
    def __init__(self):
        # Configurable parameters
        self.length = 40  # Tail length
        self.frame_width = 720  # Resized frame width
        self.min_radius = 10  # Minimum radius for object detection
        self.cap = cv.VideoCapture(0)
        
        # Create an instance of SelectColor and run it to get HSV values
        self.sc = SelectColor()
        hsv_values = self.sc.SelectColorRun()  # This runs the color selection and returns the tuple
        
        # Check if HSV values were returned (not None) and unpack them
        if hsv_values:
            self.low_H, self.low_S, self.low_V, self.high_H, self.high_S, self.high_V = hsv_values
            # Define lower and upper HSV bounds as NumPy arrays for cv.inRange
            self.lower_hsv = np.array([self.low_H, self.low_S, self.low_V])
            self.upper_hsv = np.array([self.high_H, self.high_S, self.high_V])
        else:
            print("ERROR: No HSV values returned! Using default full range.")
            # Default to full HSV range if no values are provided
            self.lower_hsv = np.array([0, 0, 0])
            self.upper_hsv = np.array([180, 255, 255])

    def TrackColor(self):
        if not self.cap.isOpened():
            print("Error: Could not open video capture")
            return

        pts = deque(maxlen=self.length)

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            # Resize frame using OpenCV
            height = int((self.frame_width / frame.shape[1]) * frame.shape[0])
            frame = cv.resize(frame, (self.frame_width, height), interpolation=cv.INTER_AREA)

            # Apply Gaussian Blur and convert to HSV
            blurred = cv.GaussianBlur(frame, (11, 11), 0)
            hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
            mask = cv.inRange(hsv, self.lower_hsv, self.upper_hsv)

            # Apply morphological operations
            mask = cv.erode(mask, None, iterations=2)
            mask = cv.dilate(mask, None, iterations=2)

            # Find contours
            contours, _ = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

            center = None
            if len(contours) > 0:
                # Find the largest contour
                c = max(contours, key=cv.contourArea)
                ((x, y), radius) = cv.minEnclosingCircle(c)
                M = cv.moments(c)
                if M["m00"] != 0:  # Avoid division by zero
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                if radius > self.min_radius:
                    cv.circle(frame, (int(x), int(y)), int(radius), (255, 0, 0), 2)  # Blue circle
                    cv.circle(frame, center, 5, (0, 0, 255), -1)  # Red dot for center
                    print(f'Coordinates: X={int(x)}, Y={int(y)}')

            pts.appendleft(center)

            # Draw the tail
            for i in range(1, len(pts)):
                if pts[i - 1] is None or pts[i] is None:
                    continue
                thickness = int(np.sqrt(self.length / float(i + 1)) * 2.5)
                cv.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)  # Red tail

            cv.imshow("Tracker", frame)
            cv.imshow("Thresholded Image", mask)

            key = cv.waitKey(10) & 0xFF  # Ensure proper key detection
            if key == ord('q')or cv.getWindowProperty("Tracker", cv.WND_PROP_VISIBLE) < 1 or cv.getWindowProperty("Thresholded Image", cv.WND_PROP_VISIBLE) < 1:
                break

        self.cap.release()
        cv.destroyAllWindows()

# Example usage
if __name__ == "__main__":
    tracker = ColorTracker()
    tracker.TrackColor()