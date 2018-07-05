#!/usr/bin/env python

import os
import sys
import cv2
import numpy as np
try:
    import tensorflow as tf
except ImportError:
    print("unable to import TensorFlow. Is it installed?")
    print("  sudo apt install python-pip")
    print("  sudo pip install tensorflow")
    sys.exit(1)

# ROS related imports
import rospy
from std_msgs.msg import String , Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Object detection module imports
import object_detection
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# SET FRACTION OF GPU YOU WANT TO USE HERE
GPU_FRACTION = 0.4

######### Set model here ############
MODEL_NAME = 'ssd_mobilenet_v1_coco_2017_11_17'
MODEL_FILE = MODEL_NAME + '.tar.gz'
# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = os.path.join('/home/drones/models/research/object_detection/data', 'mscoco_label_map.pbtxt')
    
NUM_CLASSES = 90

detection_graph = tf.Graph()
with detection_graph.as_default():
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')

## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`,
# we know that this corresponds to `airplane`.  Here we use internal utility functions,
# but anything that returns a dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Setting the GPU options to use fraction of gpu that has been set
config = tf.ConfigProto()
config.gpu_options.per_process_gpu_memory_fraction = GPU_FRACTION
sess = tf.Session(graph = detection_graph, config = config)
# Detection
with detection_graph.as_default():
	with tf.Session(graph=detection_graph,config=config) as sess2:
		class detector:

			def __init__(self):
				self.image_pub = rospy.Publisher("debug_image",Image, queue_size=1)
				self.object_pub = rospy.Publisher("objects", Detection2DArray, queue_size=1)
				self.bridge = CvBridge()
				self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_cb, queue_size=1, buff_size=2**24)

			def image_cb(self, data):
				objArray = Detection2DArray()
				cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
				image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
				image_np = np.asarray(image)
				image_np_expanded = np.expand_dims(image_np, axis = 0)
				image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
				boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
				scores = detection_graph.get_tensor_by_name('detection_scores:0')
				classes = detection_graph.get_tensor_by_name('detection_classes:0')
				num_detections = detection_graph.get_tensor_by_name('num_detections:0')
				(boxes, scores, classes, num_detections) = sess.run([boxes, scores, classes, num_detections], feed_dict={image_tensor:image_np_expanded})
				objects = vis_util.visualize_boxes_and_labels_on_image_array(
					image,
					np.squeeze(boxes),
					np.squeeze(classes).astype(np.int32),
					np.squeeze(scores),
					category_index,
					use_normalized_coordinates=True,
					line_thickness=2)

				# objArray.detections = []
				# objArray.header = data.header
				# object_count = 1

				# for i in range(len(objects)):
				# 	object_count+=1
				# 	objArray.detections.append(self.object_predict(objects[i], data.header, image_np, cv_image))
				img = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
				image_out = Image()
				image_out = self.bridge.cv2_to_imgmsg(img, "bgr8")
				image_out.header = data.header
				self.image_pub.publish(image_out)

			def object_predict(self, object_data, header, image_np, image):
				image_height, image_width, channels = image.shape
				obj = Detection2D()
				obj_hypothesis = ObjectHypothesisWithPose()
				object_id = object_data[0]
				object_score = object_data[1]
				dimensions = object_data[2]
				obj.header = header
				obj_hypothesis.id = object_id
				obj_hypothesis.score = object_score
				obj.results.append(obj_hypothesis)
				obj.bbox.size_y = int((dimensions[2] - dimensions[0])*image_height)
				obj.bbox.size_x = int((dimensions[2]-dimensions[1] )*image_width)
				obj.bbox.center.x = int((dimensions[1] + dimensions [2])*image_height/2)
				obj.bbox.center.y = int((dimensions[0] + dimensions[2])*image_width/2)
				return obj


def main(args):
	rospy.init_node('object_detection', anonymous=True)
	obj = detector()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print("ShutDown")
	cv2.destroyAllWindows()

if __name__=="__main__":
	main(sys.argv)