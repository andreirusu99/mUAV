#!/usr/bin/env python3
import threading
import time

import cv2
import rospy
from flask import Flask, Response
from std_msgs.msg import Bool

from src.sensors import Camera as cam

# image to be sent to the Ground Station
SEND_FPS = 20
FRAME_TIME = 1.0 / SEND_FPS
last_send_loop = 0.0

DETECTION_MAX_FPS = 2
DETECTION_FRAME_TIME = 1.0 / DETECTION_MAX_FPS
last_detection_loop = 0.0

# Server configuration
HOST_IP = "192.168.137.113"
HOST_PORT = 60500

FRAME_WRITE_LOCATION = "/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/live"

# Create the Flask object
app = Flask(__name__)

# init ROS node in separate thread
threading.Thread(target=lambda: rospy.init_node('FlaskServer', disable_signals=True)).start()

SAVE_FRAME = True


def writeFramesToDisk(pub):
    global last_detection_loop, SAVE_FRAME
    while True:
        if not SAVE_FRAME or time.time() - last_detection_loop < DETECTION_FRAME_TIME:
            continue

        last_detection_loop = time.time()

        detection_started = rospy.get_param("/run/detection_started")

        if detection_started:

            if cam.FRAME is not None:
                frame = cam.FRAME.copy()
                # save frame to disk for detection
                cv2.imwrite(FRAME_WRITE_LOCATION + '/frame.jpg', frame)

                # signal that a new frame has been saved
                pub.publish(Bool(True))

                # flag to false to wait until another frame is requested
                SAVE_FRAME = False


def handleFrameRequested(data):
    global SAVE_FRAME
    SAVE_FRAME = True


def encodeFrame():
    global last_send_loop
    while True:

        # send valid frames at SEND_FPS frames per second
        if time.time() - last_send_loop < FRAME_TIME:
            continue

        last_send_loop = time.time()

        frame = cam.FRAME.copy()
        # nearest interpolation since quality is not important for the video stream
        resized = cv2.resize(frame, (410, 308), interpolation=cv2.INTER_NEAREST)
        _, encoded = cv2.imencode(".jpg", resized)

        # Output image as a byte array
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
               bytearray(encoded) + b'\r\n')


@app.route("/live_feed")
def streamFrames():
    return Response(encodeFrame(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == '__main__':
    try:
        # publish a value after a frame has been saved
        frame_share_pub = rospy.Publisher('FrameSaved', Bool, queue_size=1)

        # listen for frame requests
        rospy.Subscriber('FrameRequested', Bool, handleFrameRequested)

        # start the camera thread
        camera_thread = threading.Thread(target=lambda: cam.captureFrames())
        camera_thread.daemon = True
        camera_thread.start()

        # start the frame sharing thread
        frame_thread = threading.Thread(target=writeFramesToDisk, args=[frame_share_pub])
        frame_thread.daemon = True
        frame_thread.start()

        # start the Flask Web Application
        app.run(host=HOST_IP, port=HOST_PORT, use_reloader=False, threaded=True)

        cam.video_capture.release()
        camera_thread.join(1)

    except rospy.ROSInterruptException as error:
        pass
