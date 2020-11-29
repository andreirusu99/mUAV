import os

import pathlib

import matplotlib
import tensorflow as tf

import time

import numpy as np

from PIL import Image
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


def retrieve_images():
    image_paths = []

    for filename in os.listdir("C:\\Users\\andre\\mUAV\\SSDMobileNetV2_Test\\data\\images"):
        image_paths.append("C:\\Users\\andre\\mUAV\\SSDMobileNetV2_Test\\data\\images\\" + filename)

    return image_paths


IMAGE_PATHS = retrieve_images()
print("Running on {} images: {}".format(len(IMAGE_PATHS), IMAGE_PATHS))

MODEL_NAME = 'efficientdet_d1_coco17_tpu-32'
PATH_TO_MODEL_DIR = "C:\\Users\\andre\\mUAV\\SSDMobileNetV2_Test\\model\\" + MODEL_NAME


# Download labels file
def download_labels(filename):
    base_url = 'https://raw.githubusercontent.com/tensorflow/models/master/research/object_detection/data/'
    label_dir = tf.keras.utils.get_file(fname=filename,
                                        origin=base_url + filename,
                                        untar=False)
    label_dir = pathlib.Path(label_dir)
    return str(label_dir)


LABEL_FILENAME = 'mscoco_label_map.pbtxt'
PATH_TO_LABELS = "C:\\Users\\andre\\mUAV\\SSDMobileNetV2_Test\\data\\labels\\" + LABEL_FILENAME

PATH_TO_SAVED_MODEL = PATH_TO_MODEL_DIR + "\\saved_model"

print('Loading model...')
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
    return np.array(Image.open(path))


category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)

for image_path in IMAGE_PATHS:
    print('Running inference for {}... '.format(image_path))

    start_time = time.time()
    image_np = load_image_into_numpy_array(image_path)

    # Things to try:
    # Flip horizontally
    # image_np = np.fliplr(image_np).copy()

    # Convert image to grayscale
    # image_np = np.tile(
    #     np.mean(image_np, 2, keepdims=True), (1, 1, 3)).astype(np.uint8)

    # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
    input_tensor = tf.convert_to_tensor(image_np)

    # The model expects a batch of images, so add an axis with `tf.newaxis`.
    input_tensor = input_tensor[tf.newaxis, ...]

    # input_tensor = np.expand_dims(image_np, 0)

    # perform inference on the input batch
    detections = detect_fn(input_tensor)

    end_time = time.time()

    print("Inference took {:.2f} seconds".format(end_time - start_time))

    # All outputs are batches tensors.
    # Convert to numpy arrays, and take index [0] to remove the batch dimension.
    # We're only interested in the first num_detections.
    num_detections = int(detections.pop('num_detections'))
    detections = {key: value[0, :num_detections].numpy()
                  for key, value in detections.items()}
    detections['num_detections'] = num_detections

    # detection_classes should be ints.
    detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

    image_np_with_detections = image_np.copy()

    viz_utils.visualize_boxes_and_labels_on_image_array(
        image_np_with_detections,
        detections['detection_boxes'],
        detections['detection_classes'],
        detections['detection_scores'],
        category_index,
        use_normalized_coordinates=True,
        max_boxes_to_draw=100,
        min_score_thresh=.3,
        line_thickness=2
    )

    plt.figure()
    plt.imshow(image_np_with_detections)
    print('Done')

plt.show()
