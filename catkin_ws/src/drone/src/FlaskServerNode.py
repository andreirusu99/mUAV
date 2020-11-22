#!/usr/bin/env python3
import threading
import time

import cv2
import rospy
from flask import Flask, Response

import Camera as cam

# Image frame sent to the Flask object
FRAME = None
MAX_FRAME_TIME = cam.MAX_FRAME_TIME
last_cam_sent = 0.0

# Server configuration
HOST_IP = "192.168.137.113"
HOST_PORT = 60500

# Create the Flask object for the application
app = Flask(__name__)

# init ROS node in separate thread
threading.Thread(target=lambda: rospy.init_node('FlaskServer', disable_signals=True)).start()


def encodeFrame():
    global FRAME, last_cam_sent
    while True:
        FRAME = cam.FRAME
        # send valid frames, only after MAX_FRAME_TIME
        if FRAME is None or time.time() - last_cam_sent < MAX_FRAME_TIME:
            continue

        _, encoded = cv2.imencode(".jpg", FRAME)
        last_cam_sent = time.time()

        # Output image as a byte array
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
               bytearray(encoded) + b'\r\n')


@app.route("/live_feed")
def streamFrames():
    return Response(encodeFrame(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == '__main__':
    try:
        # start the camera thread
        camera_thread = threading.Thread(target=lambda: cam.captureFrames())
        camera_thread.daemon = True
        camera_thread.start()

        # start the Flask Web Application
        rospy.loginfo("{}: Starting Flask Server on {}:{}".format(rospy.get_caller_id(), HOST_IP, HOST_PORT))
        app.run(host=HOST_IP, port=HOST_PORT, use_reloader=False, threaded=False)

        cam.video_capture.release()
        camera_thread.join(1)

    except rospy.ROSInterruptException as error:
        pass
