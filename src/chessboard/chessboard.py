import cv2
import numpy as np

# Assume perfect camera
camera_matrix = np.eye(3)
dist_coeff = None

# Calculate the coordinates of the cornes in the object (3D) frame
def calc_object_points(pattern_size, square_size):
    chess_columns = pattern_size[0]
    chess_rows = pattern_size[1]

    # The chessboard is on the z=0 plane. We have to omit the z coordinate
    object_points = np.zeros( ( chess_rows,chess_columns, 3), dtype=np.float32 )

    # Fill the coordinates
    for r in range(chess_rows):
        for c in range(chess_columns):
            object_points[r][c] = [c*square_size, r*square_size, 0]
        
    # Reshape
    object_points = object_points.reshape( -1, 3)

    #print(object_points.shape)
    #print(object_points)

    return object_points

def calc_object_points_without_z(pattern_size, square_size):
    object_points = calc_object_points(pattern_size, square_size)
    object_points = object_points[:, :2]
    return object_points

def draw_object_frame(image, homography):

    image_with_axes = cv2.drawFrameAxes(image, homography, np.zeros((3,)), np.zeros((3,)), 1)
    cv2.imshow('Distorted Image', image_with_axes)
    cv2.waitKey(0)

def rotation():
    # Calculate the rotation and translation vectors
    ret, rvec, tvec = cv2.solvePnP(obj_points, image_points, camera_matrix, dist_coeff)
    print(rvec)
    print(tvec)

    # Calculate the rotation matrix
    R, _ = cv2.Rodrigues(rvec)
    print(R)

    # Define a point in 3D space
    point_3d = np.array([0,0,0]).reshape(-1,1)

    # Convert to homogeneous coordinates
    point_3d_homogeneous = np.vstack((point_3d, 1))

###########################################################################################

# Load the distorted image
distorted_img = cv2.imread("printed_chessboard_reduced.jpg")
cv2.imshow('Distorted Image', distorted_img)
cv2.waitKey(0)

# Define the number of corners to detect
pattern_size = (7,5)

# Convert to grayscale
distorted_gray = cv2.cvtColor(distorted_img, cv2.COLOR_BGR2GRAY)

# Find the corners of the chessboard
ret, corners = cv2.findChessboardCorners(distorted_gray, pattern_size, None)

# Draw the corners
if ret:
    # Refine corner locations to subpixel accuracy
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(distorted_gray, corners, (5, 5), (-1, -1), criteria)

    cv2.drawChessboardCorners(distorted_img, pattern_size, corners, ret)
    cv2.imshow('Distorted Image', distorted_img)
    cv2.waitKey(0)

# Define a pattern of points in the 3D space corresponding to the corners 
# Each point has 3 coordinates, we choose to put them in a plane where z=0
square_size = 0.01
obj_points = calc_object_points_without_z(pattern_size, square_size)

# Define the corresponding points in the image
image_points = corners.astype(np.float32).reshape(-1,2)

homography, _ = cv2.findHomography(obj_points, image_points)
print(homography)

frame_points = np.array([[0, 0],   # Origin
                          [0.01, 0],   # X-axis
                          [0, 0.01]])   # Y-axis
obj_points = frame_points

obj_points_homogeneous = np.hstack( (obj_points, np.ones((obj_points.shape[0], 1)) ) ).T
print(obj_points_homogeneous.shape)

obj_points_projected = np.dot( homography, obj_points_homogeneous )
print(obj_points_projected.shape)
obj_points_projected = obj_points_projected[:-1,:]
print(obj_points_projected.shape)
obj_points_projected = obj_points_projected.astype(int)
for point in obj_points_projected.T:
    cv2.circle(distorted_img, tuple(point), 5, (0, 255, 0), -1)
cv2.imshow('Distorted Image', distorted_img)
cv2.waitKey(0)


