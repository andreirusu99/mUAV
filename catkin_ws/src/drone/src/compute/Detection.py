#!/usr/bin/env python3

import jetson.inference
import jetson.utils

# IT WORKS

# load as network a pre-trained MobileNetV2 SSD, using TensorRT
net = jetson.inference.detectNet("ssd-inception-v2", threshold=0.3)

# take as input the CSI camera
camera = jetson.utils.gstCamera(640, 360, "csi://0")

while True:
    img, width, height = camera.CaptureRGBA(zeroCopy=1)

    detections = net.Detect(img, width, height)

    np_img = jetson.utils.cudaToNumpy(img, width, height, 4)

    print(np_img)
    print(len(detections))
