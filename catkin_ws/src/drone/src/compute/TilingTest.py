#!/usr/bin/env python3
import time

import cv2
from src.compute import Detector as detector

NAME = 'street1_rpi'
IMAGE_IN = f'/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/images/{NAME}.jpg'  # 71 people
W_TILES = 6
H_TILES = 4

detector.load_net('ssd-mobilenet-v2', threshold=0.35)

image = cv2.imread(IMAGE_IN)
# image = cv2.resize(image, (1632, 1232), interpolation=cv2.INTER_LANCZOS4)

start_time = time.time()
overlay, detections = detector.run_inference(image, W_TILES, H_TILES)
end_time = time.time()

IMAGE_OUT = f'/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/random/{NAME}_' \
            f'{W_TILES}x{H_TILES}_{len(detections)}.jpg'

print(f"{W_TILES}x{H_TILES}: {round((end_time - start_time) * 1000)}ms, {len(detections)} people")

cv2.imwrite(IMAGE_OUT, overlay)
