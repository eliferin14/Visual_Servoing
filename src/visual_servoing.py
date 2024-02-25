import cv2
import time
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
        frame = utils.downsample_image(frame, 0.7)
        center = ( int(frame.shape[1]/2), int(frame.shape[0]/2) )

        # Extract green channel
        green = utils.hsv_mask(frame, 53, 128, 128, 100, 255, 255)

        # Detect circles
        circles = utils.detect_circles(green)

        # Print the coordinates of the click if available, then reset
        global clicked_x, clicked_y, clicked_point
        if clicked_x != -1 and clicked_y != -1:

            # Define the clicked point
            clicked_point = (clicked_x, clicked_y)

            # Calculate the angle using the center and the clicked point
            angle = utils.get_angle(center, clicked_point, 0)

            #print("Clicked coordinates: ({}, {})".format(clicked_x, clicked_y))
            print(f"angle: {angle}")

            # Reset clicked coordinates
            clicked_x = -1
            clicked_y = -1

        # Display the captured frame
        utils.draw_circles(frame, circles)
        utils.draw_cross(frame, clicked_point, 50, (0,0,255), 2)
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