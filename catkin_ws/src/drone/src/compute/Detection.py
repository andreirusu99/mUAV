#!/usr/bin/env python3

import os
import time

import cv2
import numpy as np
from PIL import Image

import jetson.inference
import jetson.utils

NET = None

IMAGE_PATH = '/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/images/queue1.jpg'


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
    print("Loading network...")
    NET = jetson.inference.detectNet(net_name, threshold=threshold)
    print("Success! Network loaded.")


def run_inference(image_path):
    image_name = image_path.split(sep='/')[-1]
    print('Running inference for {}... '.format(image_name))

    # open image with OpenCV
    image_np = cv2.imread(image_path)
    image_np = cv2.resize(image_np, (1280, 720))  # resize to 720p
    # image_np = cv2.blur(image_np, (3, 3))  # add blur

    # apply Adaptive Histogram Equalisation
    # img_hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)  # convert image from BGR to HSV
    # img_hsv[:, :, 2] = cv2.equalizeHist(img_hsv[:, :, 2])  # Histogram equalisation on the V-channel
    image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)  # convert image from HSV to RGB for detection
    image_np = np.asarray(image_np)  # save as NumPy array

    width_tiles = 4
    height_tiles = 3
    tile_width = image_np.shape[1] // width_tiles
    tile_height = image_np.shape[0] // height_tiles
    channels = image_np.shape[2]

    all_detections = []
    print('Tile Width {}, Tile Height {}, {} channels'.format(tile_width, tile_height, channels))

    start_inference = time.time()
    # detect on all tiles separately
    for tile_index, tile in enumerate(tile_image(image_np, width_tiles, height_tiles)):
        print('Running inference for {}, tile {} '.format(image_name, tile_index + 1))

        # copy the tile (numpy array) to GPU as cudaImage
        image_cuda = jetson.utils.cudaFromNumpy(tile)

        # perform detection on the cudaImage, add overlays
        detections = NET.Detect(image_cuda)
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

        # print out performance info
        # NET.PrintProfilerTimes()

        # convert back to BGR and numpy array
        # tile_overlay = jetson.utils.cudaToNumpy(image_cuda, tile_width, tile_height, channels)
        # tile_overlay = cv2.cvtColor(tile_overlay, cv2.COLOR_RGB2BGR)
        # tile_overlay = np.asarray(tile_overlay)

        # save to file
        # print("Saving...")
        # cv2.imwrite(
        #     "/home/andrei/Desktop/mUAV/catkin_ws/src/drone/out/images/" + str(tile_index + 1) + '_' + image_name,
        #     tile_overlay)
        # print("Saved!")

    end_inference = time.time()
    print("Inference took {:.1f}ms".format((end_inference - start_inference) * 1000))

    # convert back to BGR
    image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)

    # filter out the detections based on rules
    all_detections = list(filter(filter_detection, all_detections))

    print("{} detected objects".format(len(all_detections)))
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

    # save the overlayed image
    cv2.imwrite("/home/andrei/Desktop/mUAV/catkin_ws/src/drone/out/images/" + image_name, image_np)


if __name__ == '__main__':
    # load a pre-trained network, using TensorRT
    load_net("ssd-mobilenet-v2", threshold=0.4)

    # run inference (detection) on a supplied image
    run_inference(IMAGE_PATH)
