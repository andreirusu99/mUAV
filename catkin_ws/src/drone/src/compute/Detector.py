#!/usr/bin/env python3

import os
import time

import cv2
import numpy as np
from PIL import Image

import jetson.inference
import jetson.utils

NET = None


def tile_image(image, xPieces, yPieces):
    im = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

    imgwidth, imgheight = im.size

    width = imgwidth // xPieces
    height = imgheight // yPieces
    crops = []
    for i in range(0, yPieces):
        for j in range(0, xPieces):
            box = (j * width, i * height, (j + 1) * width, (i + 1) * height)
            crop = np.array(im.crop(box))
            crop = cv2.cvtColor(crop, cv2.COLOR_RGB2BGR)
            crops.append(crop)

    return crops


def filter_detection(detection):
    return detection.ClassID == 1 and 100 < detection.Area < 10000


def load_net(net_name, threshold):
    global NET
    NET = jetson.inference.detectNet(net_name, threshold=threshold)


def run_inference(image, width_tiles, height_tiles):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # convert image to RGB for detection
    image_np = np.asarray(image)  # save as NumPy array

    tile_width = image_np.shape[1] // width_tiles
    tile_height = image_np.shape[0] // height_tiles
    channels = image_np.shape[2]

    all_detections = []

    # detect on all tiles separately
    for tile_index, tile in enumerate(tile_image(image_np, width_tiles, height_tiles)):

        # copy the tile (numpy array) to GPU as cudaImage
        image_cuda = jetson.utils.cudaFromNumpy(tile)

        # perform detection on the cudaImage, do not add overlays
        detections = NET.Detect(image_cuda, overlay='none')
        # add these detections to the list of all detections
        all_detections.extend(detections)

        # add offsets to the boxes to translate them to the final image
        # row and column with respect to the final image
        row = tile_index // width_tiles
        col = tile_index % width_tiles
        for detection in detections:
            detection.Left += col * tile_width
            detection.Top += row * tile_height

            detection.Right += col * tile_width
            detection.Bottom += row * tile_height
            # center computation is done automatically inside the detectNet.Detection object

    # convert back to BGR
    image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)

    # filter out the detections based on rules (person, box area, etc.)
    all_detections = list(filter(filter_detection, all_detections))

    for detection in all_detections:
        # cv2.putText(image_np,
        #             str(round(detection.Confidence * 100, 1)) + '%',
        #             (int(detection.Left), int(detection.Top)),
        #             fontFace=cv2.FONT_HERSHEY_SIMPLEX,
        #             fontScale=0.5,
        #             color=(255, 255, 255),
        #             thickness=2)

        cv2.rectangle(image_np,
                      (int(detection.Left), int(detection.Top)),
                      (int(detection.Right), int(detection.Bottom)),
                      color=(255, 255, 255),
                      thickness=2)

    return image_np, all_detections

# load_net("ssd-mobilenet-v2", 0.4)
#
# image, det = run_inference(
#     cv2.resize(
#         cv2.imread('/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/images/nyc.jpg'),
#         (1920, 1080),
#         cv2.INTER_LANCZOS4),
#     4, 3)
#
# cv2.imwrite(
#     "/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/out/random/"
#     + str(time.time()) + '_' + str(len(det)) + '.jpg',
#     image)
