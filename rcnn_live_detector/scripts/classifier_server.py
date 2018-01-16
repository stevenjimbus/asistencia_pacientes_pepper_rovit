#!/usr/bin/env python
#jcarlos 2289
# 21/06/2016

import sys
import rospy
from std_msgs.msg import String
from subprocess import call
from rcnn_live_detector.msg import Prediction
from rcnn_live_detector.msg import PredictionsList
from rcnn_live_detector.srv import *


def add_path(path):
    if path not in sys.path:
        sys.path.insert(0, path)

# Add caffe to PYTHONPATH
caffe_path = '/home/steven/py-faster-rcnn/caffe-fast-rcnn/python/' 
add_path(caffe_path)

# Add lib to PYTHONPATH
lib_path = '/home/steven/py-faster-rcnn/lib/'
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
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

CLASSES = [] 
proposalData = {}
dictList = []
lista =[]

#caffeNet = config_model()

#caffeModel Definition
cfg.TEST.HAS_RPN = True  # Use RPN for proposals
prototxt = os.path.join('/home/steven/py-faster-rcnn/models/modelbotella20octubre/botella/test.prototxt')
caffemodel = os.path.join('/home/steven/py-faster-rcnn/models/modelbotella20octubre/model.caffemodel')
if not os.path.isfile(caffemodel):
    raise IOError(('{:s} not found.\nDid you run ./data/script/'
                    'fetch_faster_rcnn_models.sh?').format(caffemodel))
#caffe.set_mode_cpu()
caffe.set_mode_cpu()
net = caffe.Net(prototxt, caffemodel, caffe.TEST)

lbl_path = '/home/steven/py-faster-rcnn/botellaTags.txt' 
lbl_file = open(lbl_path, 'r')

with open(lbl_path) as f:
    content = f.readlines()

for tag in content:
    CLASSES.append(tag.replace("\n", ""))


#callback function that waits until the centinel topic recibe an msg to beggin the classification
def centinel_cb(req):
    global lista
    print('~~~~~~~~~~~~~~~~Beggining of Tagging Procedure.~~~~~~~~~~~~~~~~~~~')
  	#del lista[:]
    lista = []
    demo(net, req.image)
    print('PyCaffe done JCarlos.')
    #plt.show()
    msgList = PredictionsList()
    msgList.n = len(lista)
    msgList.predictions = lista
    

    
    

    #return the predictions list to the service
    return imageTaggerResponse(msgList)

    

def vis_detections(im, class_name, dets, thresh=0.5):
    """Draw detected bounding boxes."""
        
    inds = np.where(dets[:, -1] >= thresh)[0]
    if len(inds) == 0:
        return

    im = im[:, :, (2, 1, 0)]
    #fig, ax = plt.subplots(figsize=(12, 12))
    #ax.imshow(im, aspect='equal')

    
    for i in inds:
        bbox = dets[i, :4]
        score = dets[i, -1]
        
        boundingBox = []
        boundingBox.append(float(bbox[0]))
        boundingBox.append(float(bbox[1]))
        boundingBox.append(float(bbox[2]))
        boundingBox.append(float(bbox[3]))  
        
        print("**************") 
        print(class_name)
        print(score)
        print(boundingBox)  

             

        msg = Prediction()
        msg.label = class_name
        msg.score = score
        msg.bbox = boundingBox
        lista.append(msg)



def demo(net, image_or):
    """Detect object classes in an image using pre-computed object proposals."""

    #im_file = image_name
    im = CvBridge().imgmsg_to_cv2(image_or, "bgr8") #cv2.imread(im_file)  
    # Detect all object classes and regress object bounds
    timer = Timer()
    timer.tic()
    scores, boxes = im_detect(net, im)
    timer.toc()
 
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
        vis_detections(im, cls, dets, thresh=CONF_THRESH)




def classifier_server():
    rospy.init_node('classifier_server')
    #caffeNet = config_model()
    s = rospy.Service('tag_Service', imageTagger, centinel_cb)
    print "Ready to tag an Image."
     
    rospy.spin()
   
if __name__ == '__main__':
    try:
        classifier_server()
       
    except rospy.ROSInterruptException:
        pass
