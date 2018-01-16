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
#include "naoqi_bridge_msgs/JointAnglesWithSpeed.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"

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
ros::Subscriber speechObject_sub;
ros::ServiceClient clienteClasificacion;
ros::ServiceClient clienteProfundidad;

//const std::string RECEIVE_RGB_DEPTH_TOPIC_NAME = "/cv_camera/image_raw_th";
const std::string RECEIVE_RGB_DEPTH_TOPIC_NAME = "/speechObject_sub_topic";//"/rgb/rect_out";



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
  sensor_msgs::Image variableWaitForRGBimage;
  sensor_msgs::ImagePtr punteroWaitForRGBimage;
  variableWaitForRGBimage= *(ros::topic::waitForMessage<sensor_msgs::Image>("/naoqi_driver_node/camera/front/image_rect_color", ros::Duration(10)));
  punteroWaitForRGBimage = boost::make_shared<sensor_msgs::Image>(variableWaitForRGBimage);
  return punteroWaitForRGBimage;
}

sensor_msgs::ImagePtr punteroImagenDEPTH(){
  sensor_msgs::Image variableWaitForDEPTHimage= *(ros::topic::waitForMessage<sensor_msgs::Image>("/naoqi_driver_node/camera/depth/image_rect", ros::Duration(10)));
  sensor_msgs::ImagePtr punteroWaitForDEPTHimage = boost::make_shared<sensor_msgs::Image>(variableWaitForDEPTHimage);
  return punteroWaitForDEPTHimage;
}




std::vector<std::string> naoJointNames(2) ;
std::vector<float> naoJointAngles(2) ;
float speed;
uint8_t rel ;
ros::Publisher joint_angles_pub;
naoqi_bridge_msgs::JointAnglesWithSpeed msg;

double degreesToRadians(const double& degrees)
{
    return degrees*3.14159/180.0;
}

double RadiansToDegress(const double& Radians)
{
    return Radians*180.0/3.14159;
}


void setHeadPosition(float Yaw, float Pitch){
  
    naoJointAngles.clear();
    naoJointAngles.push_back(degreesToRadians(Yaw)); //"HeadYaw";
    naoJointAngles.push_back(degreesToRadians(Pitch));//"HeadPitch";
    speed = 0.1;
    rel = 0;
    msg.joint_names = naoJointNames;
    msg.joint_angles = naoJointAngles;  // float [] -In Radians (must be array)
    msg.speed = speed; // float 
    msg.relative = rel; // unit8
    joint_angles_pub.publish(msg); 
   

}


std::vector<double> QuatToEuler(const tf::Quaternion& quat )
{
  std::vector<double> vector;
  tf::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  vector.push_back(RadiansToDegress(roll)); 
  vector.push_back(RadiansToDegress(pitch));  
  vector.push_back(RadiansToDegress(yaw));  
  return vector;
}

tf::Quaternion EulerYPRToQuat(float Yaw, float Pitch, float Roll)
{
  tf::Matrix3x3 obs_mat;
  obs_mat.setEulerYPR(degreesToRadians(Yaw),degreesToRadians(Pitch),degreesToRadians(Roll));
  tf::Quaternion q_tf;
  obs_mat.getRotation(q_tf);
  float SetTorsoX = q_tf.getX();
  float SetTorsoY = q_tf.getY();
  float SetTorsoZ = q_tf.getZ();
  float SetTorsoW = q_tf.getW();
  std::cout << "SetTorsoX:" << SetTorsoX << "SetTorsoY:" << SetTorsoY << "SetTorsoZ:" << SetTorsoZ << "SetTorsoW:" << SetTorsoW << std::endl;  
  return q_tf;
}


void RotateBaseYPR(float yaw, float pitch, float roll){
  geometry_msgs::PoseStamped MoveBaseSimpleGoalMessage;
  tf::Quaternion q_tf;
  q_tf = EulerYPRToQuat(yaw,pitch,roll);
  MoveBaseSimpleGoalMessage.header.frame_id = "torso";
  MoveBaseSimpleGoalMessage.pose.orientation.x = q_tf.getX();
  MoveBaseSimpleGoalMessage.pose.orientation.y = q_tf.getY();
  MoveBaseSimpleGoalMessage.pose.orientation.z = q_tf.getZ();
  MoveBaseSimpleGoalMessage.pose.orientation.w = q_tf.getW();
  moveBase_pub.publish(MoveBaseSimpleGoalMessage);


}



//void callBack(const rcnn_live_detector::paquete_imagenes input_package)
void callBack(const std_msgs::String::ConstPtr& messageObject)
{
  system("rm /home/steven/Desktop/debug/*");
  system("python /home/steven/importantPythonScripts/deactivateReflexes.py");
  std::string MessageFromCallBack = messageObject->data.c_str();
  sensor_msgs::ImagePtr in_image_RGB;
  sensor_msgs::ImagePtr CorreccionCVimagePuntero;
  nav_msgs::Odometry OdometriaPepper;
  cv_bridge::CvImageConstPtr ptrGuardarRGBInicio;
  int contadorImagenes;
  contadorImagenes = 0;

  char QRpathmapAreaRGBinicio[512];
  char QRpathmapAreaDrawCenteredLine[512];
  char QRpathmapAreaDetectedBBOX[512];


  
  

  /*DECLARACION DEL SERVICIO PARA OBTENER LOS -TAGS-*/
  rcnn_live_detector::imageTagger       servicioClasificacion;
  //rgbdimage_sub_.shutdown();
  std::cout <<"***********************************************" <<std::endl;
  ROS_INFO("Received object message.");

  bool keepMapping = true;
  int parametroBase = 0;
  int parametroHead = -5;


  while(keepMapping) {
    if(parametroHead > 5){
      parametroBase++;
      parametroHead=-5;
      RotateBaseYPR(140,0,0);
      sleep(1);
      }
    contadorImagenes++;
    std::cout << "********************************************!"<< std::endl;
    std::cout << "*******Estoy en mapAreaQR************!"<< std::endl;
    std::cout << "********************************************!"<< std::endl;
    float anguloYawHead = 22*parametroHead;// -110 a 110 con un paso de 22 grados
    
    setHeadPosition(anguloYawHead,9);
    if (parametroHead == -5){
      if (parametroBase > 0){
        sleep(7);
      }
      else{
        sleep(3);
      }
    }
    else{
      sleep(1.2);
    }
    ptrGuardarRGBInicio = takeImageRGBandSave();
    cv::Mat matrizRGBInicio =ptrGuardarRGBInicio->image;
    snprintf(QRpathmapAreaRGBinicio, 512, "/home/steven/Desktop/debug/QRmapAreaRGBinicio%03d.jpg", contadorImagenes);       
    cv::imwrite(QRpathmapAreaRGBinicio, matrizRGBInicio );

    in_image_RGB   = punteroImagenRGB();//Tomar la imagen
    std::cout <<"@@@@@@@@@@@@FLAG-Image kaboom  @@@@@@@@@@@@@@" <<std::endl;
    servicioClasificacion.request.image = *in_image_RGB;//Construir llamada al servicio

    if (clienteClasificacion.call(servicioClasificacion))
    {
       
       ROS_INFO("Received information from rcnn_tagger node.");
       //Leer el mensaje del tipo PredictionsList.msg
       int n = servicioClasificacion.response.tags.n;
       std::vector<rcnn_live_detector::Prediction> listOfPredictions = servicioClasificacion.response.tags.predictions;

       std::string NotObjectDetectedFLag = "NotObjectDetected";     
       int longitudlistOfPredictions=listOfPredictions.size();

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




         if(tag.compare(MessageFromCallBack) == 0) {        
          std::cout << "Se ha identificado el objeto: "<< MessageFromCallBack<< std::endl;


          //**********Obtener odometria de pepper**********************
          OdometriaPepper= *(ros::topic::waitForMessage<nav_msgs::Odometry>("/naoqi_driver_node/odom", ros::Duration(10)));
          float quatX = OdometriaPepper.pose.pose.orientation.x;//orientacion X del torso de pepper
          float quatY = OdometriaPepper.pose.pose.orientation.y;//orientacion Y del torso de pepper
          float quatZ = OdometriaPepper.pose.pose.orientation.z;//orientacion Z del torso de pepper
          float quatW = OdometriaPepper.pose.pose.orientation.w;//orientacion W del torso de pepper          
          tf::Quaternion q(quatX, quatY, quatZ, quatW);// x y z w --- construir el quaternion para convertirlo a yawTorso pitchTorso roll Torso
          std::vector<double> QuatTorso;
          QuatTorso=QuatToEuler(q);
          double rollTorsoInicial=QuatTorso[0];
          double pitchTorsoInicial=QuatTorso[1];
          double yawTorsoInicial=QuatTorso[2];      
          std::cout << "rollTorsoInicial: " << rollTorsoInicial << ", pitchTorsoInicial: " << pitchTorsoInicial << ", yawTorsoInicial: " << yawTorsoInicial << std::endl;
          
          //**********Calcular correccion basado en la imagen*********************
          /*
          int heightImageCV = imageCV.cols;
          int widthImageCV = imageCV.rows;
          std::cout << "height de imagen: " << heightImageCV << std::endl;
          std::cout << "width de imagen: " << widthImageCV << std::endl;
          */
          float CorreccionAnguloImage;
          float promBBOX;
          promBBOX = (bbox.at(0)+bbox.at(2))/2;
          std::cout << "promBBOX" << promBBOX << std::endl;
          
          //CorreccionAnguloImage = -0.1725*promBBOX+27.6;//320x240
          CorreccionAnguloImage = 27.6 - promBBOX*55.2/640;//640x480
          std::cout << "bbox.at(0): " << bbox.at(0) << ", bbox.at(2): " << bbox.at(2)  << std::endl;
          std::cout << "CorreccionAnguloImage: " << CorreccionAnguloImage  << std::endl;



          //double rollTorsoFinal=0;
          //double pitchTorsoFinal=0;
          double rollTorsoFinal=rollTorsoInicial;
          double pitchTorsoFinal=pitchTorsoInicial;
          std::cout << "yawTorsoInicial  :  " << yawTorsoInicial << std::endl;
          std::cout << "anguloYawHead  :  " <<anguloYawHead  << std::endl;
          std::cout << "CorreccionAnguloImage  :  " <<CorreccionAnguloImage  << std::endl;
          double yawTorsoFinal=yawTorsoInicial + anguloYawHead + CorreccionAnguloImage;
          std::cout << "yawTorsoFinal  :  " <<yawTorsoFinal  << std::endl;
          std::cout << "rollTorsoFinal: " << rollTorsoFinal << ", pitchTorsoFinal: " << pitchTorsoFinal << ", yawTorsoFinal: " << yawTorsoFinal << std::endl;
          


          double alinearTorsoAnguloYaw = yawTorsoFinal - yawTorsoInicial; 
          std::cout << "alinearTorsoAnguloYaw: " << alinearTorsoAnguloYaw << std::endl;
          

          std::cout << "yawTorsoInicial: " << yawTorsoInicial << std::endl;
          std::cout << "anguloYawHead: " << anguloYawHead << std::endl;
          std::cout << "alinearTorsoAnguloYaw = yawTorso + anguloYawHead + CorreccionAnguloImage " << std::endl;
          std::cout << "Alinear Torso a Angulo: " << alinearTorsoAnguloYaw << std::endl;



          RotateBaseYPR(alinearTorsoAnguloYaw, pitchTorsoFinal, rollTorsoFinal);
          setHeadPosition(0,0);
          sleep(3);
          CorreccionCVimagePuntero = punteroImagenRGB();
          cv_bridge::CvImageConstPtr CorreccionCVimageBridge = cv_bridge::toCvShare(CorreccionCVimagePuntero, sensor_msgs::image_encodings::BGR8);
          cv::Mat CorreccionCVimageMat = CorreccionCVimageBridge->image;
          cv::line(CorreccionCVimageMat, cv::Point(320,5), cv::Point(320,315), cv::Scalar(0,0,255), 1, 8);

          snprintf(QRpathmapAreaDetectedBBOX, 512, "/home/steven/Desktop/debug/QRmapAreaDetectedBBOX%03d.jpg", contadorImagenes);     
          cv::imwrite( QRpathmapAreaDetectedBBOX, imageCV);

          snprintf(QRpathmapAreaDrawCenteredLine, 512, "/home/steven/Desktop/debug/QRmapAreaDrawCenteredLine%03d.jpg", contadorImagenes);
          cv::imwrite(QRpathmapAreaDrawCenteredLine, CorreccionCVimageMat);          
          DetectedObject_pub.publish(messageObject);
          keepMapping = false;
          };
         
       }
       
       std::cout << ss.str();

       std_msgs::String msg;
       msg.data = ss.str();
       centinel_pub_.publish(msg);

       
       sensor_msgs::ImagePtr imagenComoMensaje = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageCV).toImageMsg();
       
       /*DECLARACION DEL REQUEST AL SERVICIO: servicioClasificacion*/
     }
    else
    {
       ROS_ERROR("Failed to call service tag_Service");
       //return 1;
     }

     parametroHead++;
     std::cout << "Parametro head" << parametroHead << std::endl;

     /*
     if(parametroHead > 5){
      parametroBase++;
      parametroHead=-5;
      RotateBaseYPR(140,0,0);
      sleep(1);
      }
      */

      if(parametroBase > 4){
        keepMapping = false;
        std::cout << "Exit Mapping process"<< std::endl;
      }

  }


}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "mapAreaQR");
  ROS_INFO("inicio de nodo mapAreaQR");
  nh_ptr_ = boost::make_shared<ros::NodeHandle>();

  //Cliente de servicio de clasificacion
  clienteClasificacion = nh_ptr_->serviceClient<rcnn_live_detector::imageTagger>("/qr_service_project_panda");



  //Suscribirse al reconocimiento de voz
  speechObject_sub = nh_ptr_->subscribe<std_msgs::String>("/go_map_QR_code", 1000, callBack);



  

  //Publicar a mover base 
  moveBase_pub = nh_ptr_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

  //publicar a las articulaciones de la cabeza
  joint_angles_pub = nh_ptr_->advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("/joint_angles" ,1);

  //publicar mensaje que contiene objeto reconocido
  centinel_pub_ = nh_ptr_->advertise<std_msgs::String>("/centinela", 1);

  //publicar el objeto reconocido
  DetectedObject_pub = nh_ptr_->advertise<std_msgs::String>("DetectedQrCode", 1);

  

  /*******Declarar nombres de articulaciones******/
  naoJointNames.clear();
  naoJointNames.push_back("HeadYaw");
  naoJointNames.push_back("HeadPitch");
  /*****************************************/

  ros::spin();

  return 0;

}