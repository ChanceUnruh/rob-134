# Color Tracking Project

This project consists of two Python scripts that work together to allow users to select a color range from a live video feed using HSV (Hue, Saturation, Value) color space and then track objects of that color in real-time with a visual tail effect. The scripts use OpenCV (`cv2`) for image processing and video capture.

- **`hsv_thresholding.py`**: Provides a tool to interactively select a color range by adjusting HSV thresholds using trackbars.
- **`color_tracker.py`**: Tracks objects matching the selected HSV range in a live video feed and draws a trail behind them.

## Features
- Interactive HSV color range selection with real-time feedback.
- Real-time object tracking based on the selected color range.
- Visual tail effect showing the tracked object's movement.
- Adjustable parameters like tail length, frame size, and minimum object size.

## Prerequisites
- Python 3.x
- OpenCV (`opencv-python`)
- NumPy (`numpy`)

## Usage

### Step 1: Select a Color Range
Run `python hsv_thresholding.py` to define the HSV range of the color you want to track:
- Two windows will appear:
  - **Video Capture**: Shows the live feed from your webcam.
  - **HSV Values**: Shows the thresholded image based on the current HSV range.
- Adjust the trackbars (`Low H`, `High H`, `Low S`, `High S`, `Low V`, `High V`) to isolate the desired color.
- Press `s` to save the HSV values and exit, or `q` to quit without saving.
- The saved HSV values are returned and used by the tracker.

### Step 2: Track the Color
Run `python color_tracker.py` to start tracking objects of the selected color:
- The script will first launch the color selection tool (from Step 1).
- After saving the HSV values (or closing the selection tool), it begins tracking:
  - **Tracker**: Displays the live feed with a blue circle around the detected object, a red dot at its center, and a red tail showing its path.
  - **Thresholded Image**: Shows the binary mask of the tracked color.
- Press `q` or close either window to stop the tracker.
- The coordinates of the tracked object's center are printed to the console.

## Scripts

### `hsv_thresholding.py`
This script defines the `SelectColor` class, which allows users to select an HSV color range using trackbars.

#### Class: `SelectColor`
- **Purpose**: Interactively select a color range from a live video feed.
- **Constructor (`__init__`)**:
  - Initializes HSV bounds (0-180 for Hue, 0-255 for Saturation and Value).
  - Sets up the webcam feed via `cv.VideoCapture(0)`.
- **Key Methods**:
  - `SetupWindows()`: Creates windows and trackbars for HSV adjustment.
  - `SelectColorRun()`: Runs the color selection loop, displaying the live feed and thresholded result. Returns the selected HSV values as a tuple `(low_H, low_S, low_V, high_H, high_S, high_V)` when `s` is pressed, or `None` if quit without saving.
  - Trackbar callbacks (e.g., `low_H_trackbar`): Update HSV bounds dynamically.
  - `cleanup()`: Releases the webcam and closes windows.

### `color_tracker.py`
This script defines the `ColorTracker` class, which tracks objects of a specified color and visualizes their movement.

#### Class: `ColorTracker`
- **Purpose**: Track objects matching a user-defined HSV range in real-time.
- **Constructor (`__init__`)**:
  - Configurable parameters: `length` (tail length), `frame_width` (resized frame width), `min_radius` (minimum object size).
  - Runs `SelectColor` to get HSV values and sets up lower/upper HSV bounds.
  - Falls back to full HSV range if no values are provided.
- **Key Methods**:
  - `TrackColor()`: Main tracking loop:
    - Resizes the frame, applies Gaussian blur, and converts to HSV.
    - Creates a mask using `cv.inRange` with the selected HSV bounds.
    - Applies morphological operations (erosion and dilation) to refine the mask.
    - Finds contours, identifies the largest object, and calculates its center and radius.
    - Draws a blue circle around the object, a red dot at its center, and a red tail of previous positions.
    - Displays the tracking window and thresholded mask.

## Example Output
- During color selection: Adjust trackbars to isolate a red object (e.g., a pen).
- During tracking: A blue circle outlines the pen, a red dot marks its center, and a red tail follows its movement across the frame.

## Notes
- Ensure good lighting for accurate color detection.
- The webcam index (default `0`) may need adjustment if you have multiple cameras (e.g., change to `cv.VideoCapture(1)`).
- The tail length and minimum radius can be tweaked in `color_tracker.py` for different use cases, such as tracking smaller or faster objects.
