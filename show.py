#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pyrealsense2 as rs
import numpy as np
import cv2
from copy import deepcopy
import random
import os

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# air shot
color_grounds = []
while len(color_grounds) < 30:
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue
    color_grounds.append(None)
    cv2.waitKey(1)

if not os.path.exists("color_ground_orig.png"):
    print("orig_image does not exsit.")
    color_grounds = []
    while len(color_grounds) < 30:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        color_image = np.asanyarray(color_frame.get_data())
        color_grounds.append(deepcopy(color_image))
        cv2.waitKey(1)

    color_ground = np.mean(color_grounds, axis=0) # float64 のまま使う
    color_ground_orig = np.array(color_ground, np.uint8)
    cv2.imwrite("color_ground_orig.png", color_ground_orig)

else:
    print("orig_image exsit.")
    color_ground_orig = cv2.imread("color_ground_orig.png")
    color_ground = np.array(color_ground_orig, np.float64)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        color_frame = np.asanyarray(color_frame.get_data())
        
        color_image = np.array(color_frame, np.float64)
        color_image = np.abs(color_ground - color_image)
        color_image = np.array(color_image, np.uint8)
        
        color_image = deepcopy(cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY))
        color_image = np.where(color_image > 20, 255, 0) # 二値化
        color_image = np.array(color_image, np.uint8)
        
        ret,thresh = cv2.threshold(color_image,127,255,0)
        imgEdge,contours,hierarchy = cv2.findContours(thresh, 1, 2)
        
        color_image = deepcopy(cv2.cvtColor(color_image, cv2.COLOR_GRAY2RGB))
        
        for cnti, cnt in enumerate(contours):
            if cv2.contourArea(cnt) < 800:
                continue
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            c = [63, 127, 191]
            r,g,b = random.choice(c), random.choice(c), random.choice(c)
            color_image = deepcopy(cv2.drawContours(color_image,[box],0,(b,g,r),3))
        
        cv2.rectangle(color_image, (167, 118), (473, 302), (255, 0, 0), thickness=1, lineType=cv2.LINE_AA)
        cv2.rectangle(color_frame, (167, 118), (473, 302), (255, 0, 0), thickness=1, lineType=cv2.LINE_AA)
        
        # images = np.hstack((color_frame, color_image))
        images = color_frame
        
        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        # cv2.namedWindow('depth_ground_orig', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('depth_ground_orig', color_ground_orig)
        
        cv2.waitKey(10)

finally:
    # Stop streaming
    pipeline.stop()
