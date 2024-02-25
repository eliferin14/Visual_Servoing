import cv2
import time
import numpy as np
import utils

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

##############################################################
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
    motor_center = (0,0)
    end_center = (0,0)

    # FPS indicator
    fps = 0

    # Error
    error_previous = 0

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
        frame = utils.downsample_image(frame, 0.7)

        # Isolate green and detect circle
        green = utils.hsv_mask(frame, 53, 128, 128, 100, 255, 255)
        green_circles = utils.detect_circles(green)

        # Isolate white and detect circle
        white = utils.hsv_mask(frame, 0, 0, 255, 255, 10, 255)
        white_circles = utils.detect_circles(white, minRadius=20, maxRadius=30)

        # Center of the white and green circles
        if white_circles is not None:
            motor_center = (white_circles[0][0][0], white_circles[0][0][1])
        if green_circles is not None:
            end_center = (green_circles[0][0][0], green_circles[0][0][1])

        # Print the coordinates of the click if available, then reset
        global clicked_x, clicked_y, clicked_point
        if clicked_x != -1 and clicked_y != -1:

            # Define the clicked point
            clicked_point = (clicked_x, clicked_y)

            # Reset clicked coordinates
            clicked_x = -1
            clicked_y = -1

        # Compute the target and measured angles
        target_angle = utils.get_angle(motor_center, clicked_point, 0)
        measured_angle = utils.get_angle(motor_center, end_center, 0)
        error_measured = target_angle - measured_angle
        error_angle = utils.lowpass(error_measured, error_previous, 0.3)
        error_previous = error_angle

        # Display the captured frame
        utils.draw_circles(frame, green_circles)
        utils.draw_circles(frame, white_circles)
        utils.draw_cross(frame, clicked_point, 50, (0,0,255), 2)
        cv2.line(frame, np.round(motor_center).astype("int"), clicked_point, (0,0,255), 2)
        cv2.line(frame, np.round(motor_center).astype("int"), np.round(end_center).astype("int"), (255,0,0), 2)
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Error: {error_angle:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
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