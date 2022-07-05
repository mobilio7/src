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

TH_L = 0
TH_H = 15
WING = 15
CONTRAST = 90

DIST_MAX = 20.0

# We may have to modify these (just assumption)
FOV = (60) * (PI/180)     # Field of View in [rad]
BAR_LEN = 0.5             # bar length in [m]
HH, WW = 480, 640         # Height, Width of image [pixels]
##################

#img = cv2.imread(IMG_FILE)
#gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#HH, WW = gray.shape # height, width of image
F_PXL = (HH / 2) / tan(FOV / 2) # focal length in pixels

####### min filter ######
def min_filter(gray, kernel_size=3):
    pad_size = (kernel_size - 1) // 2
    HH, WW = gray.shape
    gray_padded = cv2.copyMakeBorder(gray, 0, pad_size, 0, pad_size, cv2.BORDER_CONSTANT, 255)
    output = gray.copy()
    for xx in range(HH):
        for yy in range(WW):
            min_value = gray_padded[xx:xx + kernel_size, yy:yy + kernel_size].min()
            output[xx, yy] = min_value
    return output


######## bar filter ########
def filter_bar(gray, th_l=TH_L, th_h=TH_H, wing=WING, contrast=CONTRAST):
    HH, WW = gray.shape

    black_pixels = (TH_L < gray) * (gray < TH_H)
    black_pixel_coord = [(xx, yy) for xx in range(HH) for yy in range(WW) if black_pixels[xx][yy]]

    bar_pixels_coord = []
    for xx, yy in black_pixel_coord:
        if yy < WING or WW - 1 - yy < WING:
            continue
        min_side = min(gray[xx][yy - WING], gray[xx][yy + WING])
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
#def get_bar_location(center, height, WW_crop, cylindrical=False):
def get_bar_location(center, height, WW_crop):

    FOV_W = FOV * WW_crop / WW
    #print("FOV, WW_crop, WW = ", FOV, WW_crop, WW)
    dx = center[0] - WW_crop//2

    ang = FOV_W * dx / WW_crop             # positive = CCW
    #print("FOV_W, dx, WW_crop, ang = ", FOV_W, dx, WW_crop, ang)
    dist = BAR_LEN * F_PXL / height # distance to the bar

    xpos, ypos = dist * cos(ang), dist * sin(ang)
    return (ang, dist, xpos, ypos)

    #if cylindrical:
    #    return ang, dist # (CCW angle, distance)
    #else:
    #    return dist * cos(ang), dist * sin(ang) # (front disposition, right disposition)


class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.cropped_image_pub = rospy.Publisher("cropped_image_topic",Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

        self.last_position = (DIST_MAX, DIST_MAX)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # Cut image (horizontally & vertically)
        img_bottom, img_top = 0.2, 0.8
        img_left, img_right = 0.3, 0.7
        img_bottom, img_top = int(HH * img_bottom), int(HH * img_top)
        img_left, img_right = int(WW * img_left), int(WW * img_right)
        cv_image = cv_image[img_bottom:img_top, img_left:img_right]

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)        
        gray = min_filter(gray)
        out_img = filter_bar(gray)
        out_img = cv2.medianBlur(out_img, 7)

        # get contour
        try:
            #contours = cv2.findContours(out_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            contours = cv2.findContours(out_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours[0] if len(contours) == 2 else contours[1]
            cntr = contours[-1]
  
            # get rotated rectangle from contour
            rot_rect = cv2.minAreaRect(cntr)
            box = cv2.boxPoints(rot_rect)
            box = np.int0(box)

            # draw rotated rectangle on copy of img
            rot_bbox = gray.copy()
            cv2.drawContours(rot_bbox, [box], 0, (255,255,255), 1)

            # get dimensions
            (center), (width,height), angle = rot_rect

            # print dimensions
            #print('length=', height)
            #print('thickness=', width)
            #print('angle=', angle)

            COLOR = (0, 255, 0)
            THICK = 1
            H_, W_ = rot_bbox.shape
            rot_bbox = cv2.cvtColor(rot_bbox, cv2.COLOR_GRAY2RGB)
            cv2.line(rot_bbox, (W_//2, 0), (W_//2, H_-1), COLOR, THICK)

            cv2.imshow('rot_bbox', rot_bbox)

            ang, dist, x_dist, y_dist = get_bar_location(center, height, cv_image.shape[1])

            if max(x_dist, abs(y_dist)) > DIST_MAX:
                 x_dist, y_dist = self.last_position
            else:
                 self.last_position = x_dist, y_dist

            #ang, dist = get_bar_location(center, height, out_img.shape[1], True)
            print("(ang, dist, x, y) = (%.2f, %.2f)" % (ang*180/PI, dist, x_dist, y_dist))
            #print("ang = %.2f [deg], dist = %.2f[m]" % (ang*180/PI, dist))

        except:
        #except cv2.error as e:
        #    print(e)
            print("No Object Detected")    
        
        out_img = cv2.cvtColor(out_img, cv2.COLOR_GRAY2RGB)
        #cv2.imshow("Cropped image", cv_image)
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
