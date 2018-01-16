/*
procesor receive roscaffe_msg with 
information about localization of objetcs

  Created on: june 1, 2017
 *      Author: jcarlos2289

*/
#include <ros/ros.h>
// PCL specific includes
//#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include "roscaffe_msgs/LocalizedPredictions.h"
#include "rcnn_live_detector/Prediction.h"
#include "rcnn_live_detector/PredictionsList.h"

#include <ros/ros.h>
#include "ros/package.h"
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
//#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <stdexcept>
#include <eigen3/Eigen/Dense>
//#include <pcl/io/pcd_io.h>

//#include <pcl/visualization/pcl_visualizer.h>

ros::Subscriber image_sub;
ros::Subscriber cluster_sub;
//ros::Publisher pub;
//ros::Publisher pub_rviz;
//ros::Publisher vis_bbox_pub;
//ros::Publisher vis_tag_pub;
//ros::Publisher tag_pub;
//ros::Publisher pub_cluster_rviz;
//ros::Publisher pub_reader_results;

std::vector<std::vector<double> > global_confidences;
std::vector<std::vector<std::string> > global_labels;
std::vector<geometry_msgs::Point> global_position;
//std::vector<cloud_tagging::Prediction> global_prediction_vector;

std::vector<geometry_msgs::Point> global_centroids_position;
//std::vector<pcl::PointCloud<pcl::PointXYZ>> global_clusterVector;
//pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud; //original cloud

int global_predictionNum, global_locationsNum, global_tagsAmount, globalClusterAmount;

void cluster_cb(rcnn_live_detector::PredictionsList input)
{
  // drawInfo();
  ROS_INFO("Received information from rcnn_tagger node.");

  int n = input.n;
  std::vector<rcnn_live_detector::Prediction> listOfPredictions = input.predictions;

  std::stringstream ss;
  ss << "Cantidad de Predicciones: " << n << std::endl;
  for (int i = 0; i < listOfPredictions.size(); ++i)
  {
    std::string tag = listOfPredictions.at(i).label;
    float score = listOfPredictions.at(i).score;
    std::vector<double> bbox = listOfPredictions.at(i).bbox;
    ss << "\nLabel: " << tag << std::endl
       << "Confidence: " << score << std::endl
       << "X1: " << bbox.at(0) << std::endl
       << "Y1: " << bbox.at(1) << std::endl
       << "X2: " << bbox.at(2) << std::endl
       << "Y2: " << bbox.at(3) << std::endl;
  }
  std::cout << ss.str();
  //ros::shutdown();
  

}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "msg_reader");
  boost::shared_ptr<ros::NodeHandle> nh_ptr_;

  nh_ptr_ = boost::make_shared<ros::NodeHandle>();

  // Create a ROS subscriber for the input point cloud
  // topicName, queueSize, callbackFunction

  //sub = nh_ptr_->subscribe ("/roscaffe/localized_predictions", 1, prediction_cb);
  ROS_INFO("Waiting for data from  /cluster_data_topic.");

  //this receive the info from the rcnn_tagger
  cluster_sub = nh_ptr_->subscribe("/cluster_data_topic", 1, cluster_cb);
  
  // Spin
  ros::spin();
 
}
