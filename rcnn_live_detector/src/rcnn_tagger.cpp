#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include "rcnn_live_detector/Prediction.h"
#include "rcnn_live_detector/PredictionsList.h"
#include "rcnn_live_detector/paquete_imagenes.h"
#include "rcnn_live_detector/imageTagger.h"
#include "rcnn_live_detector/profundidadServer.h"
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

ros::Subscriber tag_sub;
ros::Publisher image_pub_RGB;
ros::Publisher image_pub_DEPTH;
ros::Publisher centinel_pub_;
ros::Subscriber rgbdimage_sub_;
boost::shared_ptr<ros::NodeHandle> nh_ptr_;
ros::Subscriber camera_sub;
ros::Subscriber RGB_DEPTH_sub;
ros::ServiceClient clienteClasificacion;
ros::ServiceClient clienteProfundidad;

//const std::string RECEIVE_RGB_DEPTH_TOPIC_NAME = "/cv_camera/image_raw_th";
const std::string RECEIVE_RGB_DEPTH_TOPIC_NAME = "/conjunto_imagenes_rgb_depth";//"/rgb/rect_out";



cv_bridge::CvImageConstPtr takeImageRGBandSave(){
  sensor_msgs::Image variableWaitForRGBimage= *(ros::topic::waitForMessage<sensor_msgs::Image>("/naoqi_driver_node/camera/front/image_rect_color", ros::Duration(10)));
  sensor_msgs::ImagePtr punteroWaitForRGBimage = boost::make_shared<sensor_msgs::Image>(variableWaitForRGBimage);
  cv_bridge::CvImageConstPtr RGBimageCV = cv_bridge::toCvShare(punteroWaitForRGBimage, sensor_msgs::image_encodings::BGR8);
  return RGBimageCV;
}

cv_bridge::CvImageConstPtr takeImageDEPTHandSave(){
  sensor_msgs::Image variableWaitForDEPTHimage= *(ros::topic::waitForMessage<sensor_msgs::Image>("/naoqi_driver_node/camera/depth/image_rect", ros::Duration(10)));
  sensor_msgs::ImagePtr punteroWaitForDEPTHimage = boost::make_shared<sensor_msgs::Image>(variableWaitForDEPTHimage);
  cv_bridge::CvImageConstPtr DEPTHimageCV = cv_bridge::toCvShare(punteroWaitForDEPTHimage, sensor_msgs::image_encodings::TYPE_32FC1);
  return DEPTHimageCV;
}


sensor_msgs::ImagePtr punteroImagenRGB(){
  sensor_msgs::Image variableWaitForRGBimage= *(ros::topic::waitForMessage<sensor_msgs::Image>("/naoqi_driver_node/camera/front/image_rect_color", ros::Duration(10)));
  sensor_msgs::ImagePtr punteroWaitForRGBimage = boost::make_shared<sensor_msgs::Image>(variableWaitForRGBimage);
  return punteroWaitForRGBimage;
}

sensor_msgs::ImagePtr punteroImagenDEPTH(){
  sensor_msgs::Image variableWaitForDEPTHimage= *(ros::topic::waitForMessage<sensor_msgs::Image>("/naoqi_driver_node/camera/depth/image_rect", ros::Duration(10)));
  sensor_msgs::ImagePtr punteroWaitForDEPTHimage = boost::make_shared<sensor_msgs::Image>(variableWaitForDEPTHimage);
  return punteroWaitForDEPTHimage;
}







//void callBack(const rcnn_live_detector::paquete_imagenes input_package)
void callBack(const std_msgs::String messageObject)
{
  //rgbdimage_sub_.shutdown();
  std::cout <<"***********************************************" <<std::endl;
  ROS_INFO("Received Image.");
  

  cv::Mat imageRGBCV = takeImageRGBandSave()->image;
  cv::imwrite( "/home/steven/Desktop/RGB.jpg", imageRGBCV);

  cv::Mat imageDEPTHCV = takeImageDEPTHandSave()->image;
  float profundidadpixel= imageDEPTHCV.at<float> (100,100);
  std::cout << "Profundidad de pixel 100 100:____ "<< profundidadpixel << std::endl;
  cv::imwrite( "/home/steven/Desktop/DEPTH.jpg", imageDEPTHCV);

  /*OBTENER LAS IMAGENES DE RGB Y DEPTH DESDE EL KINECT THROTTLER*/
  /*LAS SIGUIENTES VARIABLES SON PUNTEROS*/
  /*
  sensor_msgs::ImagePtr in_image_RGB   = boost::make_shared<sensor_msgs::Image>(input_package.imagenRGB  );
  sensor_msgs::ImagePtr in_image_DEPTH = boost::make_shared<sensor_msgs::Image>(input_package.imagenDEPTH);   
  */

  sensor_msgs::ImagePtr in_image_RGB   = punteroImagenRGB();
  sensor_msgs::ImagePtr in_image_DEPTH = punteroImagenDEPTH();


  /*DECLARACION DEL SERVICIO PARA OBTENER LOS -TAGS-*/
  rcnn_live_detector::imageTagger       servicioClasificacion;
  rcnn_live_detector::profundidadServer servicioProfundidad;


  /*DECLARACION DEL REQUEST AL SERVICIO: servicioClasificacion*/
  servicioClasificacion.request.image = *in_image_RGB;

 
  if (clienteClasificacion.call(servicioClasificacion))
  {
     //std::cout << servicioClasificacion.response.tag << std::endl;
     ROS_INFO("Received information from rcnn_tagger node.");

     int n = servicioClasificacion.response.tags.n;
     std::vector<rcnn_live_detector::Prediction> listOfPredictions = servicioClasificacion.response.tags.predictions;
     //declara un vector de objetos tipo Prediction





     //
     std::cout << "flag 1"<< std::endl;
     std::string NotObjectDetectedFLag = "NotObjectDetected";     
     int longitudlistOfPredictions=listOfPredictions.size();
     std::cout << "flag 2"<< std::endl;


     /*+++++++++++++Activar para Jackson y Desactivar para Kinect*/
     /*
     if(listOfPredictions.at(0).label.compare(NotObjectDetectedFLag) == 0) {
        longitudlistOfPredictions=0; 
        std::cout << longitudlistOfPredictions<< std::endl; 
        n=0;
     };
     */


     cv_bridge::CvImageConstPtr imgOriginal = cv_bridge::toCvShare(in_image_RGB, sensor_msgs::image_encodings::BGR8);
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

     }


     std::cout << ss.str();

     std_msgs::String msg;
     msg.data = ss.str();
     centinel_pub_.publish(msg);

     //cv::imwrite("/home/jcarlos2289/catkin_ws/results/rcnn_live_detector/img.jpg", imageCV);
     sensor_msgs::ImagePtr imagenComoMensaje = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageCV).toImageMsg();
     image_pub_RGB.publish(imagenComoMensaje);


     /*DECLARACION DEL REQUEST AL SERVICIO: servicioClasificacion*/
     servicioProfundidad.request.image = *in_image_DEPTH;
     servicioProfundidad.request.tags  =  servicioClasificacion.response.tags;

     if (clienteProfundidad.call(servicioProfundidad) && (longitudlistOfPredictions>0))
     {
        std::vector<rcnn_live_detector::distance_msg> respuesta_de_servicio = servicioProfundidad.response.distanceslist;
        cv_bridge::CvImagePtr imagenDeServicio = cv_bridge::toCvCopy(servicioProfundidad.response.imageRespuestaDeServicio, sensor_msgs::image_encodings::TYPE_32FC1);
        //image_pub_DEPTH.publish(imagenDeServicio);


        for (int i=0; i<respuesta_de_servicio.size(); ++i){
          std::string tagNombre = listOfPredictions.at(i).label;
          std::cout << "Distancia " << tagNombre<<": "<< respuesta_de_servicio.at(i).distance << std::endl;
          }


     }
     else{
        std::cout <<"Failed to call a Profundidadservice por la condicion en IF statement" <<std::endl;
        
      }
      /*
      std::cout << "+++++++Vacio o no+++++++++ " << std::endl;
      listOfPredictions.clear();
      std::cout << listOfPredictions.empty() << std::endl;
      */
   }
  else
  {
     ROS_ERROR("Failed to call service tag_Service");
     //return 1;
   }






 








}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "rcnn_tagger");

  nh_ptr_ = boost::make_shared<ros::NodeHandle>();

  //this will publish the msg for the reader/visualzer node
  //cluster_pub_ = nh_ptr_->advertise<rcnn_live_detector::PredictionsList>("/cluster_data_topic", 1, connectCallback, disconnectCallback);

  //this receive the msg with the bounding box and the identified labels
  //tag_sub = nh_ptr_->subscribe("/identified_tags", 1, tag_cb);

  //this will send the order to beggin with the classification

  centinel_pub_ = nh_ptr_->advertise<std_msgs::String>("/centinela", 1);


  image_pub_RGB = nh_ptr_->advertise<sensor_msgs::Image>("/objectDetectedRGB", 1);
  //image_pub_DEPTH = nh_ptr_->advertise<sensor_msgs::Image>("/objectDetectedDEPTH", 1);


  clienteClasificacion = nh_ptr_->serviceClient<rcnn_live_detector::imageTagger>("tag_Service");
  clienteProfundidad   = nh_ptr_->serviceClient<rcnn_live_detector::profundidadServer>("depth_Service");

  




  //rgbdimage_sub_ = nh_ptr_->subscribe<sensor_msgs::Image>(RECEIVE_RGB_DEPTH_TOPIC_NAME, 1, callBack);


  //RGB_DEPTH_sub = nh_ptr_->subscribe<rcnn_live_detector::paquete_imagenes>(RECEIVE_RGB_DEPTH_TOPIC_NAME, 1, callBack);
  RGB_DEPTH_sub = nh_ptr_->subscribe<std_msgs::String>(RECEIVE_RGB_DEPTH_TOPIC_NAME, 1, callBack);

  ros::spin();

  return 0;
}