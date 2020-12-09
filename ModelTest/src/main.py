import os

import pathlib

import matplotlib
import tensorflow as tf
import cv2
import skimage as sk

import time
from PIL import Image
import numpy as np

import matplotlib.pyplot as plt

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils

matplotlib.use('tkagg')

# fix tf.gfile
tf.gfile = tf.io.gfile

print("Using TF {}".format(tf.__version__))

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'  # Suppress TensorFlow logging (1)
tf.get_logger().setLevel('ERROR')  # Suppress TensorFlow logging (2)

# Enable GPU dynamic memory allocation
gpus = tf.config.experimental.list_physical_devices('GPU')
for gpu in gpus:
    tf.config.experimental.set_memory_growth(gpu, True)


# takes an OpenCV image in numpy format
# and returns an array of tiles obtained
# from cropping the image
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


def retrieve_images():
    image_paths = []

    for filename in os.listdir("..\\data\\images"):
        image_paths.append("C:\\Users\\andre\\mUAV\\ModelTest\\data\\images\\" + filename)

    return image_paths


# IMAGE_PATHS = retrieve_images()
IMAGE_PATHS = ["C:\\Users\\andre\\mUAV\\ModelTest\\data\\images\\nyc.jpg"]
print("Running on {} images: {}".format(len(IMAGE_PATHS), IMAGE_PATHS))

MODEL_NAME = 'ssd_mobilenet_v2_320x320_coco17_tpu-8'
PATH_TO_MODEL_DIR = "C:\\Users\\andre\\mUAV\\ModelTest\\model\\" + MODEL_NAME


# Download labels file
def download_labels(filename):
    base_url = 'https://raw.githubusercontent.com/tensorflow/models/master/research/object_detection/data/'
    label_dir = tf.keras.utils.get_file(fname=filename,
                                        origin=base_url + filename,
                                        untar=False)
    label_dir = pathlib.Path(label_dir)
    return str(label_dir)


LABEL_FILENAME = 'mscoco_label_map.pbtxt'
PATH_TO_LABELS = "C:\\Users\\andre\\mUAV\\ModelTest\\data\\labels\\" + LABEL_FILENAME

PATH_TO_SAVED_MODEL = PATH_TO_MODEL_DIR + "\\saved_model"

print('Loading model {}'.format(MODEL_NAME))
start_time = time.time()

# Load saved model and build the detection function
detect_fn = tf.saved_model.load(PATH_TO_SAVED_MODEL)

end_time = time.time()
elapsed_time = end_time - start_time
print('Done! Took {:.2f} seconds'.format(elapsed_time))


def load_image_into_numpy_array(path):
    """Load an image from file into a numpy array.

    Puts image into numpy array to feed into tensorflow graph.
    Note that by convention we put it into a numpy array with shape
    (height, width, channels), where channels=3 for RGB.

    Args:
      path: the file path to the image

    Returns:
      uint8 numpy array with shape (img_height, img_width, 3)
    """
    image = cv2.imread(path)  # open image
    image = cv2.resize(image, (1280, 720), cv2.INTER_LANCZOS4)  # resize to 720p
    image = cv2.blur(image, (3, 3))  # blur image to better mach real sensor output
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # convert to RGB for detection
    image = sk.util.random_noise(image, mode='gaussian', var=0.05 ** 2)  # add random gaussian noise
    image = (255 * image).astype(np.uint8)  # convert back to [0-255] range
    return np.array(image)


category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)

for image_path in IMAGE_PATHS:
    print('Running inference for {}... '.format(image_path))

    start_time = time.time()
    image_np = load_image_into_numpy_array(image_path)
    end_preprocessing = time.time()

    print('Preprocessing took {:.1f} ms'.format((end_preprocessing - start_time) * 1000))

    plt.imsave("..\\results\\" + MODEL_NAME + "\\" + image_path.split("\\")[-1], image_np)

    width_tiles = 4
    height_tiles = 3
    detections = []
    for tile_no, tile in enumerate(tile_image(image_np, width_tiles, height_tiles)):
        print('Running inference for {}, tile {} '.format(image_path, tile_no + 1))
        start_time = time.time()

        # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
        input_tensor = tf.convert_to_tensor(tile)

        # The model expects a batch of images, so add an axis with `tf.newaxis`.
        input_tensor = input_tensor[tf.newaxis, ...]

        # perform inference on the input batch
        detections.append(detect_fn(input_tensor))

        end_time = time.time()

        print("Inference took {:.1f} ms".format((end_time - start_time) * 1000))

        plt.figure()
        # save each tile separately
        plt.imsave("..\\results\\" + MODEL_NAME + "\\" + str(tile_no + 1) + '_' + image_path.split("\\")[-1],
                   tile)

    image_np_with_detections = image_np.copy()

    # NOT WORKING CORRECTLY YET
    # TODO: scale the boxes to their tiles
    for i in range(width_tiles * height_tiles):
        # All outputs are batches tensors.
        # Convert to numpy arrays, and take index [0] to remove the batch dimension.
        # We're only interested in the first num_detections.
        num_detections = int(detections[i].pop('num_detections'))
        detections[i] = {key: value[0, :num_detections].numpy()
                         for key, value in detections[i].items()}
        detections[i]['num_detections'] = num_detections

        # detection_classes should be ints.
        detections[i]['detection_classes'] = detections[i]['detection_classes'].astype(np.int64)

        viz_utils.visualize_boxes_and_labels_on_image_array(
            image_np_with_detections,
            detections[i]['detection_boxes'],
            detections[i]['detection_classes'],
            detections[i]['detection_scores'],
            category_index,
            use_normalized_coordinates=True,
            max_boxes_to_draw=20,
            min_score_thresh=.4,
            line_thickness=1
        )

    plt.imsave("..\\results\\" + MODEL_NAME + "\\" + 'boxes_' + image_path.split("\\")[-1], image_np_with_detections)
    print('Done')

# plt.show()
