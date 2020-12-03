#!/usr/bin/env python3

# The unique camera image source of the system
import cv2
import time
import numpy as np

import jetson.utils

# the speed at which the FRAME object is updated
MAX_CAM_FPS = 30
MAX_FRAME_TIME = 1.0 / MAX_CAM_FPS  # seconds
last_cam_read = 0.0
CAPTURE_WIDTH = 1280
CAPTURE_HEIGHT = 720
CONV_WIDTH = 1280
CONV_HEIGHT = 720

# GStreamer Pipeline to access the Raspberry Pi camera
GSTREAMER_PIPELINE = 'nvarguscamerasrc ' \
                     '! video/x-raw(memory:NVMM), width={}, height={}, format=(string)NV12 ' \
                     '! nvvidconv flip-method=2 ' \
                     '! video/x-raw, width={}, height={}, format=(string)BGRx ' \
                     '! videoconvert ' \
                     '! video/x-raw, format=(string)BGR ' \
                     '! appsink wait-on-eos=false max-buffers=1 drop=True' \
    .format(CAPTURE_WIDTH, CAPTURE_HEIGHT, CONV_WIDTH, CONV_HEIGHT)

# Video capturing from OpenCV and GStreamer
video_capture = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)

FRAME = np.zeros((CONV_HEIGHT, CONV_WIDTH, 3), dtype=np.uint8)

CUDA_FRAME = jetson.utils.cudaFromNumpy(np.asarray(cv2.cvtColor(FRAME.copy(), cv2.COLOR_BGR2RGB)))


def captureFrames():
    global FRAME, CUDA_FRAME, last_cam_read

    while True and video_capture.isOpened():
        if time.time() - last_cam_read < MAX_FRAME_TIME:
            continue

        _, frame = video_capture.read()

        last_cam_read = time.time()

        FRAME = frame.copy()

        # store a CUDA frame for jetson inference lib
        cuda = frame.copy()
        # needs to be RGB, not BGR as captured
        cuda = cv2.cvtColor(cuda, cv2.COLOR_BGR2RGB)
        # convert to numpy array
        cuda = np.asarray(cuda)
        # save as cudaImage
        CUDA_FRAME = jetson.utils.cudaFromNumpy(cuda)

    video_capture.release()
