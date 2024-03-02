import cv2
import numpy as np

def detect_chessboard_corners(image, pattern_size):
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(image_gray, pattern_size, None)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(image_gray, corners, (5, 5), (-1, -1), criteria)
    return corners

# Load the two images
distorted_image = cv2.imread("printed_chessboard_reduced.jpg")
target_image = cv2.imread("printed_chessboard_frontal.png")

# Number of nternal corners to detect
pattern_size = (7,5)

# Detect corners
corners_distorted = detect_chessboard_corners(distorted_image, pattern_size)
corners_target = detect_chessboard_corners(target_image, pattern_size)

# Calculate homography
H, _ = cv2.findHomography(corners_distorted, corners_target)
print(H)

# Warp the distorted image to match the target
corrected_image = cv2.warpPerspective(distorted_image, H, (2*distorted_image.shape[1], 2*distorted_image.shape[0]))
cv2.imshow("Corrected", corrected_image)
cv2.waitKey(0)