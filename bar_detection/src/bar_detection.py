#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from math import pi as PI
from math import cos, sin, tan

### USER INPUT ###
IMG_FILE = "bar3.png"

CROP_RATE_HORIZONTAL = 0.25
CROP_RATE_VERTICAL = 0.25

TH_L = 0
TH_H = 15
WING = 15
CONTRAST = 90

MEDIAN_BLUR_SIZE = 7

DIST_MAX = 20.0

CENTER_LINE_COLOR = (0, 255, 0)
CENTER_LINE_THICKNESS = 1

# We may need to modify these
FOV = (60) * (PI / 180)  # Field of View in [rad]
BAR_LEN = 0.5  # bar length in [m]
HH, WW = 480, 640  # Height, Width of image [pixels]
##################

F_PXL = (HH / 2) / tan(FOV / 2)  # focal length in pixels


###### min filter (= erosion) #######
def min_filter(gray, kernel_size=3):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    return cv2.erode(gray, kernel)


######## bar filter ########
def filter_bar(gray, th_l=TH_L, th_h=TH_H, wing=WING, contrast=CONTRAST):
    HH, WW = gray.shape

    black_pixels_x, black_pixels_y = np.where(np.logical_and(TH_L < gray, gray < TH_H))

    bar_pixels_coord = []
    for xx, yy in zip(black_pixels_x, black_pixels_y):
        # if yy < WING or WW - 1 - yy < WING:
        #    continue
        # min_side = min(gray[xx][yy - WING], gray[xx][yy + WING])
        graylevel_l = gray[xx, yy - WING] if yy - WING >= 0 else 255
        graylevel_r = gray[xx, yy + WING] if yy + WING < WW else 255
        min_side = min(graylevel_l, graylevel_r)
        if min_side < gray[xx][yy]:
            continue
        min_diff = min_side - gray[xx][yy]
        if min_diff > CONTRAST:
            bar_pixels_coord.append((xx, yy))

    out = 0 * gray
    for xx, yy in bar_pixels_coord:
        out[xx][yy] = 255

    return out


###### Location Estimation #########
def get_bar_location(center, height, WW_crop):
    FOV_W = FOV * WW_crop / WW
    dx = center[0] - WW_crop // 2

    ang = FOV_W * dx / WW_crop  # positive = CCW
    dist = BAR_LEN * F_PXL / height  # distance to the bar

    return ang, dist


class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("bar_detection_topic", Image)
        self.cropped_image_pub = rospy.Publisher("cropped_image_topic", Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)

        self.last_dist = DIST_MAX
        self.no_object = False

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Cut image (horizontally & vertically)
        img_bottom, img_top = int(HH * CROP_RATE_VERTICAL), int(HH * (1.0 - CROP_RATE_VERTICAL))
        img_left, img_right = int(WW * CROP_RATE_HORIZONTAL), int(WW * (1.0 - CROP_RATE_HORIZONTAL))
        cv_image = cv_image[img_bottom:img_top, img_left:img_right]

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = min_filter(gray)
        out_img = filter_bar(gray)
        out_img = cv2.medianBlur(out_img, MEDIAN_BLUR_SIZE)

        # get contour
        try:
            # contours = cv2.findContours(out_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            contours = cv2.findContours(out_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours[0] if len(contours) == 2 else contours[1]
            cntr = contours[-1]

            # get rotated rectangle from contour
            rot_rect = cv2.minAreaRect(cntr)
            box = cv2.boxPoints(rot_rect)
            box = np.int0(box)

            # draw rotated rectangle on copy of img
            rot_bbox = gray.copy()
            cv2.drawContours(rot_bbox, [box], 0, (255, 255, 255), 1)

            # get dimensions
            (center), (width, height), angle = rot_rect

            H_, W_ = rot_bbox.shape
            rot_bbox = cv2.cvtColor(rot_bbox, cv2.COLOR_GRAY2RGB)
            cv2.line(rot_bbox, (W_ // 2, 0), (W_ // 2, H_ - 1), CENTER_LINE_COLOR, CENTER_LINE_THICKNESS)

            cv2.imshow('rot_bbox', rot_bbox)

            ang, dist = get_bar_location(center, height, cv_image.shape[1])
            exec("dist = self.last_dist" if dist > DIST_MAX else "self.last_dist = dist")
            x_dist, y_dist = dist * cos(ang), dist * sin(ang)

            print("(ang, dist, x, y) = (%.2f deg, %.2f m, %.2f m, %.2f m)" % (ang * 180 / PI, dist, x_dist, y_dist))
            self.no_object = False

        except:
            if not self.no_object:
                print("No Object Detected")
                self.no_object = True
            rot_bbox = gray.copy()
            H_, W_ = rot_bbox.shape
            rot_bbox = cv2.cvtColor(rot_bbox, cv2.COLOR_GRAY2RGB)
            cv2.line(rot_bbox, (W_ // 2, 0), (W_ // 2, H_ - 1), CENTER_LINE_COLOR, CENTER_LINE_THICKNESS)
            cv2.imshow('rot_bbox', rot_bbox)

        out_img = cv2.cvtColor(out_img, cv2.COLOR_GRAY2RGB)
        cv2.imshow("Image window", out_img)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(out_img, "bgr8"))
            self.cropped_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)