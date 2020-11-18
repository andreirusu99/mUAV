#!/usr/bin/env python3
import threading
import time

import cv2
import rospy
from flask import Flask, Response
from std_msgs.msg import String

# Image frame sent to the Flask object
video_frame = None
MAX_CAM_FPS = 20
MAX_FRAME_TIME = 1.0 / MAX_CAM_FPS  # seconds
last_cam_sent = 0.0
last_cam_read = 0.0

# GStreamer Pipeline to access the Raspberry Pi camera
GSTREAMER_PIPELINE = 'nvarguscamerasrc ' \
                     '! video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12 ' \
                     '! nvvidconv flip-method=2 ' \
                     '! video/x-raw, width=640, height=360, format=(string)BGRx ' \
                     '! videoconvert ' \
                     '! video/x-raw, format=(string)BGR ' \
                     '! appsink wait-on-eos=false max-buffers=1 drop=True'

HOST_IP = "192.168.137.113"
HOST_PORT = 60500

# Create the Flask object for the application
app = Flask(__name__)

# init ROS node in separate thread
threading.Thread(target=lambda: rospy.init_node('FlaskServer', disable_signals=True)).start()
pub = rospy.Publisher('test_pub', String, queue_size=1)

# Video capturing from OpenCV and GStreamer
video_capture = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)


def captureFrames():
    global video_frame, video_capture, last_cam_read

    while True and video_capture.isOpened():
        if time.time() - last_cam_read < MAX_FRAME_TIME:
            continue

        return_key, frame = video_capture.read()
        last_cam_read = time.time()

        if not return_key:
            break

        video_frame = frame.copy()

        key = cv2.waitKey(30) & 0xff
        if key == 27:
            break

    video_capture.release()


def encodeFrame():
    global video_frame, last_cam_sent
    while True:
        if video_frame is None or time.time() - last_cam_sent < MAX_FRAME_TIME:
            continue

        last_cam_sent = time.time()

        return_key, encoded_image = cv2.imencode(".jpg", video_frame)

        if not return_key:
            continue

        # Output image as a byte array
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
               bytearray(encoded_image) + b'\r\n')


@app.route("/video")
def streamFrames():
    return Response(encodeFrame(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == '__main__':
    main_thread = threading.Thread(target=captureFrames)
    main_thread.daemon = True
    main_thread.start()

    # start the Flask Web Application
    # HOST_IP = os.environ['ROS_IP']
    rospy.loginfo("{}: Starting Flask Server on {}:{}".format(rospy.get_caller_id(), HOST_IP, HOST_PORT))
    app.run(host=HOST_IP, port=HOST_PORT, use_reloader=False, threaded=True)
