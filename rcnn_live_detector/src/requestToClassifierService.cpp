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

  ros::init(argc, argv, "RequestaClassifier");
  ROS_INFO("inicio de nodo RequestaClassifier");
  nh_ptr_ = boost::make_shared<ros::NodeHandle>();


  rcnn_live_detector::imageTagger       servicioClasificacion;
  clienteClasificacion = nh_ptr_->serviceClient<rcnn_live_detector::imageTagger>("tag_Service");


  sensor_msgs::Image variableWaitForRGBimage= *(ros::topic::waitForMessage<sensor_msgs::Image>("/naoqi_driver_node/camera/front/image_rect_color", ros::Duration(5)));
  sensor_msgs::ImagePtr punteroWaitForRGBimage = boost::make_shared<sensor_msgs::Image>(variableWaitForRGBimage);
  cv_bridge::CvImageConstPtr RGBimageCV = cv_bridge::toCvShare(punteroWaitForRGBimage, sensor_msgs::image_encodings::BGR8);
  
  




  servicioClasificacion.request.image = *punteroWaitForRGBimage;


  if (clienteClasificacion.call(servicioClasificacion))
    {
       //std::cout << servicioClasificacion.response.tag << std::endl;
       ROS_INFO("Received information from rcnn_tagger node.");

       int n = servicioClasificacion.response.tags.n;
       std::vector<rcnn_live_detector::Prediction> listOfPredictions = servicioClasificacion.response.tags.predictions;
       //declara un vector de objetos tipo Prediction

      
       std::string NotObjectDetectedFLag = "NotObjectDetected";     
       int longitudlistOfPredictions=listOfPredictions.size();
       


       /*+++++++++++++Activar para Jackson y Desactivar para Kinect*/
       
       if(listOfPredictions.at(0).label.compare(NotObjectDetectedFLag) == 0) {
          longitudlistOfPredictions=0; 
          std::cout << longitudlistOfPredictions<< std::endl; 
          n=0;
       };
       


       cv_bridge::CvImageConstPtr imgOriginal = cv_bridge::toCvShare(punteroWaitForRGBimage, sensor_msgs::image_encodings::BGR8);
       cv::Mat imageCV = imgOriginal->image;



       std::stringstream ss;
       ss << "Cantidad de Predicciones: " << n << std::endl;
       for (int i = 0; i < longitudlistOfPredictions; ++i)
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
         cv::rectangle(imageCV, cv::Point(bbox.at(0), bbox.at(1)), cv::Point(bbox.at(2), bbox.at(3)), cv::Scalar(0,0,255), 1, 8);


         std::vector<double> bboxOfObjectDetected;
         bboxOfObjectDetected.push_back(bbox.at(0));
         bboxOfObjectDetected.push_back(bbox.at(1));
         bboxOfObjectDetected.push_back(bbox.at(2));
         bboxOfObjectDetected.push_back(bbox.at(3));

         std::cout << ss.str();

         cv::imwrite( "/home/steven/Desktop/requestToProfundidadRGB.jpg", imageCV); 

       }

     }
    else
    {
       ROS_ERROR("Failed to call service tag_Service");
       //return 1;
     }








  return 0;

}