#!/usr/bin/env python3

import os

import cv2
import numpy as np
from PIL import Image

import jetson.inference
import jetson.utils


def tileImage(image, xPieces, yPieces):
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


# load as network a pre-trained network, using TensorRT
print("Loading network...")
net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
print("Success! Network loaded.")


def retrieve_images():
    image_paths = []

    for filename in os.listdir("../../data/images"):
        image_paths.append("/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/images/" + filename)

    return image_paths


print("Collecting inference images...")
IMAGE_PATHS = retrieve_images()
print("Done! Running on {} images: {}".format(len(IMAGE_PATHS), IMAGE_PATHS))

for image_path in IMAGE_PATHS:
    image_name = image_path.split(sep='/')[-1]
    print('Running inference for {}... '.format(image_name))

    # open image with OpenCV
    image_np = np.asarray(cv2.imread(image_path))
    image_np = cv2.resize(image_np, (1280, 720))

    # TODO: stitch the overlayed image tiles back into a single image
    for i, image_np in enumerate(tileImage(image_np, 3, 2)):
        print('Running inference for {}, tile {} '.format(image_name, i + 1))
        width = image_np.shape[1]
        height = image_np.shape[0]
        channels = image_np.shape[2]

        print('Width {}, Height {}, {} channels'.format(width, height, channels))

        # copy opencv image (numpy array) to GPU as cudaImage
        image_cuda = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        image_cuda = jetson.utils.cudaFromNumpy(image_cuda)

        # perform detection on the cudaImage, add overlays
        detections = net.Detect(image_cuda)

        # print out performance info
        net.PrintProfilerTimes()

        # convert back to BGR and numpy array
        image_overlay = jetson.utils.cudaToNumpy(image_cuda, width, height, channels)
        image_overlay = cv2.cvtColor(image_overlay, cv2.COLOR_RGB2BGR)
        image_overlay = np.asarray(image_overlay)

        # save to file
        print("Saving...")
        cv2.imwrite("../../data/out/images/" + str(i + 1) + '_' + image_name, image_overlay)
        print("Saved!")
