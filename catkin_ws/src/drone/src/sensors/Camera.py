#!/usr/bin/env python3

# The unique camera image source of the system
import cv2
import time

# the speed at which the FRAME object is updated
MAX_CAM_FPS = 25
MAX_FRAME_TIME = 1.0 / MAX_CAM_FPS  # seconds
last_cam_read = 0.0
CAPTURE_WIDTH = 3264
CAPTURE_HEIGHT = 2464
SAVE_WIDTH = 1632
SAVE_HEIGHT = 1232

# GStreamer Pipeline to access the Raspberry Pi camera
GSTREAMER_PIPELINE = 'nvarguscamerasrc ' \
                     '! video/x-raw(memory:NVMM), width={}, height={}, framerate=21/1, format=(string)NV12 ' \
                     '! nvvidconv flip-method=2 interpolation-method=5 ' \
                     '! video/x-raw, width={}, height={}, format=(string)BGRx ' \
                     '! videoconvert ' \
                     '! video/x-raw, format=(string)BGR ' \
                     '! appsink wait-on-eos=false max-buffers=1 drop=True' \
    .format(CAPTURE_WIDTH, CAPTURE_HEIGHT, SAVE_WIDTH, SAVE_HEIGHT)

# Video capturing from OpenCV and GStreamer
video_capture = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)

FRAME = None


def captureFrames():
    global FRAME, last_cam_read

    while True and video_capture.isOpened():
        if time.time() - last_cam_read < MAX_FRAME_TIME:
            continue

        _, frame = video_capture.read()

        last_cam_read = time.time()

        FRAME = frame.copy()

    video_capture.release()
