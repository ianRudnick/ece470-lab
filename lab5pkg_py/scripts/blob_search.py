#!/usr/bin/env python


from matplotlib.pyplot import hsv
import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0.0
beta = 0.0
tx = 0.0
ty = 0.0

# Function that converts image coord to world coord
def IMG2W(col, row):
    O_r = 240
    O_c = 320
    col_center = col - O_c
    row_center = row - O_r

    f = 0.075/59.0    # 75 mm in world = 59 pixels
    B = f

    x_c = row_center * B
    y_c = col_center * B

    T_x = 0.225
    T_y = 0.110

    x_w = x_c + T_x
    y_w = y_c + T_y

    # print("xw, yw: ", x_w, y_w)
    
    theta = np.radians(-2.26)  # degrees rotation from camera axis to world axis

    vec = np.array([x_w, y_w])
    rot = np.array([[np.cos(theta), np.sin(theta)],
                    [-np.sin(theta), np.cos(theta)]])    
    vec_rotated = np.dot(rot, vec)
    x_w_rotated = vec_rotated[0]
    y_w_rotated = vec_rotated[1]

    # print("world coords: ", x_w_rotated, y_w_rotated)
    return x_w_rotated, y_w_rotated

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 20
    params.maxArea = 1000

    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = 0.70
    params.maxCircularity = 0.85

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    # green_lower = (40, 80, 100)
    # green_upper = (90, 200, 200)

    pink1_lower = (170, 100, 100)
    pink1_upper = (180, 255, 255)
    pink2_lower = (0, 100, 100)
    pink2_upper = (10, 255, 255)
    green_lower = (10,150,150) #(yellow)
    green_upper = (50,255,255)

    # Define a mask using the lower and upper bounds of the target color
    #mask_image = cv2.inRange(hsv_image, lower, upper)
    pink1_mask = cv2.inRange(hsv_image, pink1_lower, pink1_upper)
    pink2_mask = cv2.inRange(hsv_image, pink2_lower, pink2_upper)
    green_mask = cv2.inRange(hsv_image, green_lower, green_upper)
    pink_mask = cv2.bitwise_or(pink1_mask, pink2_mask)
    mask_image = cv2.bitwise_or(pink_mask, green_mask)
    #mask_image = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)

    # ========================= Student's code ends here ===========================
    if color == "green":
        keypoints = detector.detect(green_mask)
    elif color == "pink":
        keypoints = detector.detect(pink_mask)
    else:
        keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, 0)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
