#!/usr/bin/env python3
import threading
import time

import cv2
import rospy
from flask import Flask, Response

from src.compute import ComputingNode as compute

# image to be sent to the Ground Station
FRAME = None
SEND_FPS = 25
FRAME_TIME = 1.0 / SEND_FPS
last_send_loop = 0.0

# Server configuration
HOST_IP = "192.168.137.113"
HOST_PORT = 60500

# Create the Flask object for the application
app = Flask(__name__)

# init ROS node in separate thread
threading.Thread(target=lambda: rospy.init_node('FlaskServer', disable_signals=True)).start()


def encodeFrame():
    global last_send_loop
    while True:

        # send valid frames at SEND_FPS frames per second
        if time.time() - last_send_loop < FRAME_TIME:
            continue

        last_send_loop = time.time()

        frame = compute.CUDA_FRAME
        print("Flask: {}".format(frame))

        if frame is None:
            continue

        # print(FRAME)
        # nearest interpolation since quality is not important for the video stream
        resized = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_NEAREST)
        _, encoded = cv2.imencode(".jpg", resized)

        # Output image as a byte array
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
               bytearray(encoded) + b'\r\n')


@app.route("/live_feed")
def streamFrames():
    return Response(encodeFrame(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == '__main__':
    try:

        # start the Flask Web Application
        app.run(host=HOST_IP, port=HOST_PORT, use_reloader=False, threaded=False)

    except rospy.ROSInterruptException as error:
        pass
