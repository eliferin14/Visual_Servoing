import cv2
import numpy as np
import utils


def main():
    cv2.destroyAllWindows()
    utils.capture_single_frame("frame1.jpg")

    frame = cv2.imread("frame1.jpg")

    g = utils.hsv_mask(frame, 53, 128, 128, 100, 255, 255)

    # Detect circles
    circles = utils.detect_circles(g)
    utils.draw_circles(frame, circles)

    cv2.namedWindow("Frame")
    cv2.imshow("Frame", frame)
    cv2.waitKey(0)

main()