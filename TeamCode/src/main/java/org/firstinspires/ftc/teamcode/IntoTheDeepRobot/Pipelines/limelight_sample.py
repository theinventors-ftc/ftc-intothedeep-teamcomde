import cv2 as cv
import numpy as np

def runPipeline(image, llrobot):
    # Parameters
    # llrobot = [0 for Red 1 for Blue 2 for Yellow, threshold, blur amount, minContourArea, roi_x1, roi_y1, roi_x2, roi_y2, ...]
    # color = llrobot[]
    # thresh = llrobot[1]
    # blurAmount = llrobot[2]
    # minContourArea = llrobot[3]
    # roi_x1 = llrobot[4]
    # roi_y1 = llrobot[5]
    # roi_x2 = llrobot[6]
    # roi_y2 = llrobot[7]

    color = 2
    thresh = 30
    blurAmount = 11
    minContourArea = 3000
    roi_x1 = 0
    roi_y1 = 0
    roi_x2 = 640
    roi_y2 = 480

    # Pre-Computation
    crop = image[roi_y1:roi_y2, roi_x1:roi_x2]
    img = cv.GaussianBlur(crop, (blurAmount, blurAmount), 0)
    img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)

    # Channel Splitting
    r_ch = np.array(img_rgb[:, :, 0], dtype=np.int16)
    g_ch = np.array(img_rgb[:, :, 1], dtype=np.int16)
    b_ch = np.array(img_rgb[:, :, 2], dtype=np.int16)

    # Thresholding (Red or Blue respectively)
    if color == 1:
        bin_image = np.where(
            (r_ch - g_ch * 1.7 > thresh) & (r_ch - b_ch > thresh), 255, 0
        ).astype(np.uint8)
    elif color == 0:
        bin_image = np.where(
            (b_ch - r_ch > thresh) & (b_ch - g_ch > thresh),
            255,
            0,
            ).astype(np.uint8)
    else:
        bin_image = np.where(
            (r_ch - b_ch > thresh) & (g_ch - b_ch > thresh),
            255,
            0,
            ).astype(np.uint8)

    contours_raw = cv.findContours(bin_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[
        0
    ]

    # Filter Contours based on Area
    contours = [contour for contour in contours_raw if cv.contourArea(contour) > minContourArea]

    samples = []

    for contour in contours:
        x, y, w, h = cv.boundingRect(contour)

        rect = cv.fitEllipse(contour)

        angle = rect[2]

        # if angle < -45:
        #     angle += 90

        theta = angle
        samples.append((contour, (x + w // 2, y + h // 2, theta)))

    # Calculate Answer(Right Most Specimen)
    right_most_sample = max(samples, key=lambda x: x[1][0]) if samples else (np.array([[]]), (0, 0, 0))
    right_most_pose = right_most_sample[1]

    llpython = [right_most_pose[0], right_most_pose[1], right_most_pose[2], 0, 0, 0, 0, 0]

    # print(right_most_pose[2])

    return right_most_sample[0], crop, llpython