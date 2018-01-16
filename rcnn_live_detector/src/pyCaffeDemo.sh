#!/bin/bash

#export PATH="/home/jcarlos2289/miniconda2/bin:$PATH"

#export PYTHONPATH="/home/jcarlos2289/Documentos/py-faster-rcnn/caffe-fast-rcnn/python/"

/home/jcarlos2289/Documentos/FRCN/myRCNN/myClassifier.py  --img $1  --lbl /home/jcarlos2289/Documentos/FRCN/myRCNN/PascalTags.txt  --saveTo /home/jcarlos2289/catkin_ws/results/rcnn_live_detector/

#/home/jcarlos2289/Documentos/py-faster-rcnn/Gluto.jpg
exit 0