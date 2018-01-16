#!/usr/bin/env python

import decimal
import sys
import rospy
import os
import json
import requests
from time import sleep
from StringIO import StringIO
from std_msgs.msg import String
from subprocess import call
from rcnn_live_detector.msg import Prediction
from rcnn_live_detector.msg import PredictionsList
from rcnn_live_detector.srv import *

def add_path(path):
    if path not in sys.path:
        sys.path.insert(0, path)



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



D = decimal.Decimal



#callback function that waits until the centinel topic recibe an msg to beggin the classification
def centinel_cb(req):
    global lista


    print('~~~~~~~~~~~~~~~Llamado a Jackson Clasificacion~~~~~~~~~~~~~~~~~~~')
    '''
    os.system('> /home/steven/CarpetaJSONcaffe/response.json')              

    #del lista[:]
    lista = []

    imagen_from_request = CvBridge().imgmsg_to_cv2(req.image, "bgr8")
    path_to_save_image = '/home/steven/CarpetaJSONcaffe/'
    cv2.imwrite(os.path.join(path_to_save_image, 'image_for_jackson.png'), imagen_from_request)

    print("HELLO JACKSON")
    os.system('curl --form "imagefile=@/home/steven/CarpetaJSONcaffe/image_for_jackson.png" http://172.18.33.89:5000/index > /home/steven/CarpetaJSONcaffe/response.json')
    print("BYE BYE JACKSON")
    

    with open('/home/steven/CarpetaJSONcaffe/response.json') as data_file:    
               generatedMap=  json.load(data_file)
    '''
    lista = []

    imagen_from_request = CvBridge().imgmsg_to_cv2(req.image, "bgr8")

    _, img_encoded = cv2.imencode('.png', imagen_from_request)

    url = 'http://172.18.33.118:5000/index'
    files = {'imagefile': img_encoded.tostring()}   #
    r = requests.post(url, files=files)

    generatedMap = json.loads(r.text)



    generatedMapClasses = generatedMap["classes"]
    generatedMapBBoxes  = generatedMap["bboxes"]
    generatedMapScores  = generatedMap["scores"]



     

    try:
        if len(generatedMapClasses)>0:
            for x in range(0, len(generatedMapClasses)):
                print(x)
                print(generatedMapClasses[x])
                print(generatedMapBBoxes[x])
                print((generatedMapScores[x]))
                msg = Prediction()
                msg.label = generatedMapClasses[x]
                msg.bbox = generatedMapBBoxes[x]  
                msg.score = float((generatedMapScores[x])) 
                lista.append(msg)      
    except:
        print("No hay objeto score=0")
        msg.label = "NotObjectDetected"
        msg.bbox = [10,10,40,40]  
        msg.score = float(0.5)  

    
    msgList = PredictionsList()
    msgList.n = len(lista)
    msgList.predictions = lista   

    #return the predictions list to the service

    return imageTaggerResponse(msgList)

    





def classifier_server():
    rospy.init_node('classifier_server_jackson')
    #caffeNet = config_model()
    s = rospy.Service('tag_Service', imageTagger, centinel_cb)
    print "Ready to tag an Image."
     
    rospy.spin()
   
if __name__ == '__main__':
    try:
        classifier_server()
       
    except rospy.ROSInterruptException:
        pass