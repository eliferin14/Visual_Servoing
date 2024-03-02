import cv2
import time
import math
import numpy as np
import serial

# Function to draw a cross
def draw_cross(image, point, size, color, thickness):
    x, y = point
    half_size = size // 2

    # Draw horizontal line
    cv2.line(image, (x - half_size, y), (x + half_size, y), color, thickness)
    # Draw vertical line
    cv2.line(image, (x, y - half_size), (x, y + half_size), color, thickness)

# Downsample
def downsample_image(image, scale_factor):
    # Calculate the new dimensions
    width = int(image.shape[1] * scale_factor)
    height = int(image.shape[0] * scale_factor)
    # Resize the image
    resized_image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    return resized_image

# Compute angle
def get_angle(point1, point2, offset):
    x1, y1 = point1
    x2, y2 = point2

    delta_x = x2 - x1
    delta_y = y2 - y1

    angle = math.atan2(-delta_y, delta_x) + offset
    return angle

def capture_single_frame(filename):
    # Open the video capture
    camera = cv2.VideoCapture('/dev/video0')

    # Check if the camera was opened successfully
    if not camera.isOpened():
        print("Error: Couldn't open camera.")
        return
    
    # Create a window and set the mouse callback function
    cv2.namedWindow('Frame')

    # Capture frame
    ret, frame = camera.read()

    # Check if the frame was captured successfully
    if not ret:
        print("Error: Couldn't capture frame.")
        exit
        
    # Save the file
    cv2.imwrite(filename, frame)

    # Release the capture device and close all OpenCV windows
    camera.release()
    cv2.destroyAllWindows()

# Extract and process a frae to get the green channel
def extract_green_channel(frame):
    b, g, r = cv2.split(frame)

    # Red mask
    red_threshold = 160
    red_mask = np.uint8(r < red_threshold)

    # Blue mask
    blue_threshold = 160
    blue_mask = np.uint8(b < blue_threshold)
 
    # Green masks
    green_lower_threshold = 160
    green_upper_threshold = 256
        
    # Create binary masks for both thresholds
    green_lower_mask = np.uint8(g >= green_lower_threshold)
    green_upper_mask = np.uint8(g <= green_upper_threshold)

    # Combine both masks to get the final mask
    final_mask = green_lower_mask & green_upper_mask & (red_mask | blue_mask)

    # Apply the mask
    g = cv2.bitwise_and(g, g, mask=final_mask)

    # Apply the threshold
    ret, g = cv2.threshold(g, 128, 255, cv2.THRESH_BINARY)

    return g

def hsv_mask(frame, hmin, smin, vmin, hmax, smax, vmax):
    lower_bound = np.array([hmin, smin, vmin])
    upper_bound = np.array([hmax, smax, vmax])

    # Convert to HSV color space
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame_hsv, lower_bound, upper_bound)
    
    return mask

# Hough transform to detect frames
def detect_circles(frame, minDist=300, quality=20, minRadius=0, maxRadius=0):

    # Apply Gaussian blur to reduce noise
    blurred_frame = cv2.GaussianBlur(frame, (9, 9), 2)

    # Apply Hough Transform to detect circles
    circles = cv2.HoughCircles(blurred_frame, cv2.HOUGH_GRADIENT, dp=1, minDist=minDist,
                               param1=300, param2=quality, minRadius=minRadius, maxRadius=maxRadius)

    return circles

# Draw circles on the image
def draw_circles(frame, circles):
    if circles is not None:
        #print(f"Circles detected: {circles.size} ")
        # Convert coordinates and radius to integers
        circles = np.round(circles[0, :]).astype("int")

        # Draw detected circles
        for (x, y, r) in circles:
            cv2.circle(frame, (x, y), r, (255, 0, 0), 4)


# Low pass filter
def lowpass(measure, old_value, smoothing):
    filtered_value = smoothing*old_value + (1-smoothing)*measure
    return filtered_value

# Send velocity reference to the ESP32
def send_command(ser, target):
    message = "M" + str(target) + "\n"
    ser.write(message.encode())

# Given an image containing a chessboard, return the corners
def detect_chessboard_corners(image, pattern_size):
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(image_gray, pattern_size, flags = cv2.CALIB_CB_FILTER_QUADS)
    if ret:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(image_gray, corners, (5, 5), (-1, -1), criteria)
        return corners
    else:
        return None
