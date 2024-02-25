import cv2
import time
import utils

def capture_single_frame():
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
    filename = "frame1.jpg"
    cv2.imwrite(filename, frame)

    # Release the capture device and close all OpenCV windows
    camera.release()
    cv2.destroyAllWindows()