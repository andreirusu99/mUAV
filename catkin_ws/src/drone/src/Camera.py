#!/usr/bin/env python3
import time

import cv2

FRAME = None
MAX_CAM_FPS = 20
MAX_FRAME_TIME = 1.0 / MAX_CAM_FPS  # seconds
last_cam_read = 0.0
CAPTURE_WIDTH = 1280
CAPTURE_HEIGHT = 720
CONV_WIDTH = 640
CONV_HEIGHT = 360

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


def captureFrames():
    global FRAME, video_capture, last_cam_read

    while True and video_capture.isOpened():
        if time.time() - last_cam_read < MAX_FRAME_TIME:
            continue

        _, frame = video_capture.read()
        last_cam_read = time.time()

        FRAME = frame.copy()

    video_capture.release()