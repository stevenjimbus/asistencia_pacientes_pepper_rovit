#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include "rcnn_live_detector/Prediction.h"
#include "rcnn_live_detector/PredictionsList.h"
#include "rcnn_live_detector/paquete_imagenes.h"
#include "rcnn_live_detector/imageTagger.h"
#include "rcnn_live_detector/profundidadServer.h"
#include "rcnn_live_detector/projectPandaProfundidadServer.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include "ros/package.h"
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <limits>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <math.h> 
#include <stdio.h>  
#include <cmath>
#include <tf/transform_datatypes.h>
#include "naoqi_bridge_msgs/JointAnglesWithSpeed.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"
#include <stdlib.h>

#include "geometry_msgs/PoseStamped.h"


#include <vector>
#include <math.h>
#include <ros/console.h>
#include <map>
#include "std_msgs/String.h"
#include <boost/foreach.hpp>
//#include <array>

#include <unistd.h>//para medir tiempo delay

ros::Subscriber tag_sub;
ros::Publisher image_pub_RGB;
ros::Publisher image_pub_DEPTH;
ros::Publisher centinel_pub_;
ros::Publisher moveBase_pub;
ros::Publisher DetectedObject_pub;
ros::Subscriber rgbdimage_sub_;
boost::shared_ptr<ros::NodeHandle> nh_ptr_;
ros::Subscriber camera_sub;
ros::Subscriber ReceiveDetectedObject_sub;
ros::ServiceClient clienteClasificacion;
ros::ServiceClient clienteProfundidad;






int main(int argc, char **argv)
{

  ros::init(argc, argv, "Requestaprofundidad");
  ROS_INFO("inicio de nodo Requestaprofundidad");
  nh_ptr_ = boost::make_shared<ros::NodeHandle>();
  rcnn_live_detector::projectPandaProfundidadServer servicioProfundidad;  

  clienteProfundidad   = nh_ptr_->serviceClient<rcnn_live_detector::projectPandaProfundidadServer>("/depth_service_project_panda");





  std::vector<double> bboxOfObjectDetected;
  bboxOfObjectDetected.push_back(90);
  bboxOfObjectDetected.push_back(60);
  bboxOfObjectDetected.push_back(140);
  bboxOfObjectDetected.push_back(165);


  sensor_msgs::Image variableWaitForRGBimage= *(ros::topic::waitForMessage<sensor_msgs::Image>("/naoqi_driver_node/camera/front/image_rect_color", ros::Duration(5)));
  sensor_msgs::ImagePtr punteroWaitForRGBimage = boost::make_shared<sensor_msgs::Image>(variableWaitForRGBimage);
  cv_bridge::CvImageConstPtr RGBimageCV = cv_bridge::toCvShare(punteroWaitForRGBimage, sensor_msgs::image_encodings::BGR8);
  cv::Mat imageRGBCV = RGBimageCV->image;
  cv::rectangle(imageRGBCV, cv::Point(bboxOfObjectDetected[0],bboxOfObjectDetected[1]), cv::Point(bboxOfObjectDetected[2], bboxOfObjectDetected[3]), cv::Scalar(0,0,255), 1, 8);
  cv::imwrite( "/home/steven/Desktop/RGB.jpg", imageRGBCV);  



  sensor_msgs::Image variableWaitForDEPTHimage= *(ros::topic::waitForMessage<sensor_msgs::Image>("/naoqi_driver_node/camera/depth/image_rect", ros::Duration(5)));
  sensor_msgs::ImagePtr punteroWaitForDEPTHimage = boost::make_shared<sensor_msgs::Image>(variableWaitForDEPTHimage);
  cv_bridge::CvImageConstPtr DEPTHimageCV = cv_bridge::toCvShare(punteroWaitForDEPTHimage, sensor_msgs::image_encodings::TYPE_32FC1);
  cv::Mat imageDEPTHCV = DEPTHimageCV->image;
  cv::rectangle(imageDEPTHCV, cv::Point(bboxOfObjectDetected[0],bboxOfObjectDetected[1]), cv::Point(bboxOfObjectDetected[2], bboxOfObjectDetected[3]), cv::Scalar(0,0,255), 1, 8);
  cv::imwrite( "/home/steven/Desktop/DEPTH.jpg", imageDEPTHCV);


  servicioProfundidad.request.image = *punteroWaitForDEPTHimage;
  servicioProfundidad.request.bbox  =  bboxOfObjectDetected;



  if (clienteProfundidad.call(servicioProfundidad))
  {
    std::cout <<"Respuesta de servicioProfundidad"<< std::endl;
    double distanciaHastaElObjeto;
    double cambioHeadPitch;
    distanciaHastaElObjeto = servicioProfundidad.response.distance_to_object;
    std::cout << "distanciaHastaElObjeto en acercamientoFino" << distanciaHastaElObjeto << std::endl;
  }

  else
  {
    std::cout <<"Failed to call a Profundidadservice por la condicion en IF statement" <<std::endl;
  }










  return 0;

}