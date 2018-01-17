#!/usr/bin/env python
#jcarlos 2289
# 21/06/2016

import time
import urllib2
from flask import Flask
from flask import jsonify
from flask import send_file
from flask import request
app = Flask(__name__)

import sys
from subprocess import call


def add_path(path):
	if path not in sys.path:
		sys.path.insert(0, path)

# Add caffe to PYTHONPATH
caffe_path = '/home/fgomez/py-faster-rcnn/caffe-fast-rcnn/python/' 
add_path(caffe_path)

# Add lib to PYTHONPATH
lib_path = '/home/fgomez/py-faster-rcnn/lib/'
add_path(lib_path)
from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect
from fast_rcnn.nms_wrapper import nms
from utils.timer import Timer
import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sio
import caffe, os, sys, cv2
import argparse

CLASSES = [] 
proposalData = {}
dictList = []
lista =[]


boundingBox=[]
score=[]
classres=[]

#caffeNet = config_model()

#caffeModel Definition
cfg.TEST.HAS_RPN = True  # Use RPN for proposals
prototxt = os.path.join('/home/fgomez/py-faster-rcnn/models/steven_full/test.prototxt')
caffemodel = os.path.join('/home/fgomez/ext1/datasets/steven/snaps/vgg16_frcnn_steven_iter_10000.caffemodel')
if not os.path.isfile(caffemodel):
	raise IOError(('{:s} not found.\nDid you run ./data/script/'
					'fetch_faster_rcnn_models.sh?').format(caffemodel))
#caffe.set_mode_cpu()
caffe.set_mode_gpu()
caffe.set_device(0)
cfg.GPU_ID = 0
net = caffe.Net(prototxt, caffemodel, caffe.TEST)

lbl_path = '/home/fgomez/py-faster-rcnn-webservice/tags.txt' 
lbl_file = open(lbl_path, 'r')

with open(lbl_path) as f:
	content = f.readlines()

for tag in content:
	CLASSES.append(tag.replace("\n", ""))


#callback function that waits until the centinel topic recibe an msg to beggin the classification
@app.route("/index", methods=['POST'])
def centinel_cb():
	
	if request.method=='POST':
		#imagefile = request.files.get('imagefile', '')
		#imagefile.save(secure_filename(imagefile.filename))
		f = request.files['imagefile']
		f.save("temp.jpeg")


		img = cv2.imread("temp.jpeg")
		global lista, boundingBox, score, classres
	  	#del lista[:]
		lista = []

		boundingBox=[]
		score=[]
		classres=[]

		demo(net, img)



		print boundingBox
		print score
		print classres
		return jsonify(
	        bboxes=boundingBox,
	        scores=score,
	        classes=classres
	    )

	

def vis_detections(im, class_name, dets, thresh=0.5):
	"""Draw detected bounding boxes."""
	global boundingBox, score, classres
		
	inds = np.where(dets[:, -1] >= thresh)[0]
	if len(inds) == 0:
		local_boundingBox = None
		local_score = None
		local_classres = None
		return

	im = im[:, :, (2, 1, 0)]
	#fig, ax = plt.subplots(figsize=(12, 12))
	#ax.imshow(im, aspect='equal')

	print len(inds),"detections"
	for i in inds:
		bbox = dets[i, :4]
		local_score = dets[i, -1]
		
		local_boundingBox = []
		local_boundingBox.append(float(bbox[0]))
		local_boundingBox.append(float(bbox[1]))
		local_boundingBox.append(float(bbox[2]))
		local_boundingBox.append(float(bbox[3]))

		
		local_classres = class_name

		print class_name," bbox", local_boundingBox

		imgCrop = im.copy()
		cv2.rectangle(imgCrop, (int(bbox[0]),int(bbox[1])),(int(bbox[2]),int(bbox[3])),(255,0,0))
		imgCrop = imgCrop[...,::-1]
		cv2.imwrite("sal.jpeg", imgCrop)  


		boundingBox.append(local_boundingBox)
		score.append(str(local_score))
		classres.append(local_classres)

   


def demo(net, image_or):
	"""Detect object classes in an image using pre-computed object proposals."""

	#im_file = image_name
	im = image_or 
	# Detect all object classes and regress object bounds
	timer = Timer()
	timer.tic()
	scores, boxes = im_detect(net, im)
	elapsed = timer.toc()
	print elapsed, "secs"
 
	CONF_THRESH = 0.995
	NMS_THRESH = 0.5 #controla la cantidad de proposals
	
	for cls_ind, cls in enumerate(CLASSES[1:]):
		cls_ind += 1 # because we skipped background
		cls_boxes = boxes[:, 4*cls_ind:4*(cls_ind + 1)]
		cls_scores = scores[:, cls_ind]
		dets = np.hstack((cls_boxes,
						  cls_scores[:, np.newaxis])).astype(np.float32)
		keep = nms(dets, NMS_THRESH)
		dets = dets[keep, :]
		print "Attempt to",cls
		vis_detections(im, cls, dets, thresh=CONF_THRESH)


def classifier_server():

	app.run(host='0.0.0.0')
	 
  
if __name__ == '__main__':
		classifier_server()
	   
