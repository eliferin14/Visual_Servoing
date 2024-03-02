import cv2
import numpy as np

def detect_chessboard_corners(image, pattern_size):
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(image_gray, pattern_size, flags = cv2.CALIB_CB_FILTER_QUADS)
    if ret:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(image_gray, corners, (5, 5), (-1, -1), criteria)
    return corners

# Open the video capture
camera = cv2.VideoCapture('/dev/video0')

# Check if the camera was opened successfully
if not camera.isOpened():
    print("Error: Couldn't open camera.")
    exit()

# Create a window and set the mouse callback function
cv2.namedWindow('Frame')

# Capture an image and calculate homography
ret, distorted_frame = camera.read()
target_image = cv2.imread("chessboard/frontal2.png")
# Number of nternal corners to detect
pattern_size = (7,5)
# Detect corners
corners_distorted = detect_chessboard_corners(distorted_frame, pattern_size)
corners_target = detect_chessboard_corners(target_image, pattern_size)
# Calculate homography
H, _ = cv2.findHomography(corners_distorted, corners_target)
print(H)

while True:
    # Capture frame
    ret, distorted_frame = camera.read()

    # Check if the frame was captured successfully
    if not ret:
        print("Error: Couldn't capture frame.")
        break

    # Correct perspective using homography
    corners_distorted = detect_chessboard_corners(distorted_frame, pattern_size)
    H, _ = cv2.findHomography(corners_distorted, corners_target)
    #distorted_frame = cv2.drawChessboardCorners(distorted_frame, pattern_size, corners_distorted, True)
    corrected_image = cv2.warpPerspective(distorted_frame, H, (2*distorted_frame.shape[1], distorted_frame.shape[0]))
    cv2.imshow("Frame", corrected_image)

    # Check for key press; if 'q' is pressed, exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture device and close all OpenCV windows
camera.release()
cv2.destroyAllWindows()