#TODO Define clicked point as Point object
#TODO Draw a cross where it is 
#TODO Calculate angle with the center and draw a line

import cv2
import time
import math

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


# When a click is detected, update the clicked_x and clicked_y variables
# Initialize the variables
clicked_x = -1
clicked_y = -1
clicked_point = (-1, -1)
def get_click_coordinates(event, x, y, flags, param):
    global clicked_x, clicked_y
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_x = x
        clicked_y = y

# Compute angle
def get_angle(point1, point2, offset):
    x1, y1 = point1
    x2, y2 = point2

    delta_x = x2 - x1
    delta_y = y2 - y1

    angle = math.atan2(-delta_y, delta_x) + offset
    return angle

#######################################################
def main():
    # Open the video capture
    camera = cv2.VideoCapture('/dev/video0')

    # Check if the camera was opened successfully
    if not camera.isOpened():
        print("Error: Couldn't open camera.")
        return
    
    # Create a window and set the mouse callback function
    cv2.namedWindow('Frame')
    cv2.setMouseCallback('Frame', get_click_coordinates)

    # Define the center to compute the angle
    center = (0,0)

    # FPS indicator
    fps = 0

    while True:
        # Start the time
        start_time = time.time()

        # Capture frame
        ret, frame = camera.read()

        # Check if the frame was captured successfully
        if not ret:
            print("Error: Couldn't capture frame.")
            break

        # Downsample
        frame = downsample_image(frame, 0.7)
        center = ( int(frame.shape[1]/2), int(frame.shape[0]/2) )



        # Print the coordinates of the click if available, then reset
        global clicked_x, clicked_y, clicked_point
        if clicked_x != -1 and clicked_y != -1:

            # Define the clicked point
            clicked_point = (clicked_x, clicked_y)

            # Calculate the angle using the center and the clicked point
            angle = get_angle(center, clicked_point, 0)

            #print("Clicked coordinates: ({}, {})".format(clicked_x, clicked_y))
            print(f"angle: {angle}")

            # Reset clicked coordinates
            clicked_x = -1
            clicked_y = -1

        # Display the captured frame
        draw_cross(frame, clicked_point, 50, (0,0,255), 2)
        cv2.line(frame, center, clicked_point, (0,0,255), 2)
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('Frame', frame)

        # Check for key press; if 'q' is pressed, exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        loop_time = time.time() - start_time
        fps = fps + 0.2*(1/loop_time - fps)


    # Release the capture device and close all OpenCV windows
    camera.release()
    cv2.destroyAllWindows()

main()
