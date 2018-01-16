#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include "rcnn_live_detector/Prediction.h"
#include "rcnn_live_detector/PredictionsList.h"
#include "rcnn_live_detector/paquete_imagenes.h"
#include "rcnn_live_detector/imageTagger.h"
#include "rcnn_live_detector/profundidadServer.h"
#include "rcnn_live_detector/projectPandaProfundidadServer.h"
#include "rcnn_live_detector/msgTomarObjeto.h"
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



ros::Publisher moveBase_pub;

ros::Publisher goGetTheObject;


boost::shared_ptr<ros::NodeHandle> nh_ptr_;

ros::Subscriber ReceiveMsgFromAcercarseFino;
ros::ServiceClient clienteClasificacion;
ros::ServiceClient clienteProfundidad;

//const std::string RECEIVE_RGB_DEPTH_TOPIC_NAME = "/cv_camera/image_raw_th";
const std::string RECEIVE_RGB_DEPTH_TOPIC_NAME = "/speechObject_sub_topic";//"/rgb/rect_out";



cv_bridge::CvImageConstPtr takeImageRGBandSave(){
  sensor_msgs::Image variableWaitForRGBimage= *(ros::topic::waitForMessage<sensor_msgs::Image>("/naoqi_driver_node/camera/front/image_rect_color", ros::Duration(5)));
  sensor_msgs::ImagePtr punteroWaitForRGBimage = boost::make_shared<sensor_msgs::Image>(variableWaitForRGBimage);
  cv_bridge::CvImageConstPtr RGBimageCV = cv_bridge::toCvShare(punteroWaitForRGBimage, sensor_msgs::image_encodings::BGR8);
  return RGBimageCV;
}

cv_bridge::CvImageConstPtr takeImageDEPTHandSave(){
  sensor_msgs::Image variableWaitForDEPTHimage= *(ros::topic::waitForMessage<sensor_msgs::Image>("/naoqi_driver_node/camera/depth/image_rect", ros::Duration(5)));
  sensor_msgs::ImagePtr punteroWaitForDEPTHimage = boost::make_shared<sensor_msgs::Image>(variableWaitForDEPTHimage);
  cv_bridge::CvImageConstPtr DEPTHimageCV = cv_bridge::toCvShare(punteroWaitForDEPTHimage, sensor_msgs::image_encodings::TYPE_32FC1);
  return DEPTHimageCV;
}


sensor_msgs::ImagePtr punteroImagenRGB(){
  sensor_msgs::Image variableWaitForRGBimage;
  sensor_msgs::ImagePtr punteroWaitForRGBimage;
  variableWaitForRGBimage= *(ros::topic::waitForMessage<sensor_msgs::Image>("/naoqi_driver_node/camera/front/image_rect_color", ros::Duration(5)));
  punteroWaitForRGBimage = boost::make_shared<sensor_msgs::Image>(variableWaitForRGBimage);
  return punteroWaitForRGBimage;
}

sensor_msgs::ImagePtr punteroImagenDEPTH(){
  sensor_msgs::Image variableWaitForDEPTHimage= *(ros::topic::waitForMessage<sensor_msgs::Image>("/naoqi_driver_node/camera/depth/image_rect", ros::Duration(5)));
  sensor_msgs::ImagePtr punteroWaitForDEPTHimage = boost::make_shared<sensor_msgs::Image>(variableWaitForDEPTHimage);
  return punteroWaitForDEPTHimage;
}




std::vector<std::string> naoJointNames(12) ;
std::vector<float> naoJointAngles(12) ;

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



void positionArms(int var[]){
    std::cout << "Llamado a positionArms" << std::endl;

    naoJointAngles.clear();
    naoJointAngles.push_back((var[0])); //"LShoulderRoll";
    naoJointAngles.push_back((var[1]));//"LShoulderPitch";
    naoJointAngles.push_back((var[2]));//"LElbowYaw";
    naoJointAngles.push_back((var[3]));//"LWristYaw";
    naoJointAngles.push_back(var[4]);//"LHand";
    naoJointAngles.push_back((var[5]));//"LElbowRoll";
    //Right arm
    naoJointAngles.push_back((var[6]));//"RShoulderRoll";
    naoJointAngles.push_back((var[7]));//"RShoulderPitch";
    naoJointAngles.push_back((var[8]));//"RElbowYaw";
    naoJointAngles.push_back((var[9]));//"RWristYaw";
    naoJointAngles.push_back(var[10]);//"RHand";
    naoJointAngles.push_back((var[11]));//"RElbowRoll"; 

    speed = 0.1;
    rel = 0;
    msg.joint_names = naoJointNames;
    msg.joint_angles = naoJointAngles;  // float [] -In Radians (must be array)
    msg.speed = speed; // float 
    msg.relative = rel; // unit8
    joint_angles_pub.publish(msg); 
}




void ShoulderRoll(double var1, double var2, double var3, double var4)
{
  naoJointNames.clear();
  naoJointNames.push_back("LShoulderRoll");
  naoJointNames.push_back("LHand");
  naoJointNames.push_back("RShoulderRoll");
  naoJointNames.push_back("RHand"); 

  naoJointAngles.clear();
  naoJointAngles.push_back(degreesToRadians(var1)); 
  naoJointAngles.push_back(var2);
  naoJointAngles.push_back(degreesToRadians(var3));
  naoJointAngles.push_back(var4);
  speed = 0.5;
  rel = 0;
  msg.joint_names = naoJointNames;
  msg.joint_angles = naoJointAngles;  // float [] -In Radians (must be array)
  msg.speed = speed; // float 
  msg.relative = rel; // unit8
  joint_angles_pub.publish(msg); 

}


void ShoulderPitch(double var1, double var2, double var3, double var4)
{
  naoJointNames.clear();
  naoJointNames.push_back("LShoulderPitch");
  naoJointNames.push_back("LHand");
  naoJointNames.push_back("RShoulderPitch");
  naoJointNames.push_back("RHand"); 

  naoJointAngles.clear();
  naoJointAngles.push_back(degreesToRadians(var1)); 
  naoJointAngles.push_back(var2);
  naoJointAngles.push_back(degreesToRadians(var3));
  naoJointAngles.push_back(var4);
  speed = 0.1;
  rel = 0;
  msg.joint_names = naoJointNames;
  msg.joint_angles = naoJointAngles;  // float [] -In Radians (must be array)
  msg.speed = speed; // float 
  msg.relative = rel; // unit8
  joint_angles_pub.publish(msg); 
}


void ElbowYaw(double var1, double var2, double var3, double var4)
{
  naoJointNames.clear();
  naoJointNames.push_back("LElbowYaw");
  naoJointNames.push_back("LHand");
  naoJointNames.push_back("RElbowYaw");
  naoJointNames.push_back("RHand"); 

  naoJointAngles.clear();
  naoJointAngles.push_back(degreesToRadians(var1)); 
  naoJointAngles.push_back(var2);
  naoJointAngles.push_back(degreesToRadians(var3));
  naoJointAngles.push_back(var4);
  speed = 0.1;
  rel = 0;
  msg.joint_names = naoJointNames;
  msg.joint_angles = naoJointAngles;  // float [] -In Radians (must be array)
  msg.speed = speed; // float 
  msg.relative = rel; // unit8
  joint_angles_pub.publish(msg); 
}


void WristYaw(double var1, double var2, double var3, double var4)
{
  naoJointNames.clear();
  naoJointNames.push_back("LWristYaw");
  naoJointNames.push_back("LHand");
  naoJointNames.push_back("RWristYaw");
  naoJointNames.push_back("RHand"); 

  naoJointAngles.clear();
  naoJointAngles.push_back(degreesToRadians(var1)); 
  naoJointAngles.push_back(var2);
  naoJointAngles.push_back(degreesToRadians(var3));
  naoJointAngles.push_back(var4);
  speed = 0.1;
  rel = 0;
  msg.joint_names = naoJointNames;
  msg.joint_angles = naoJointAngles;  // float [] -In Radians (must be array)
  msg.speed = speed; // float 
  msg.relative = rel; // unit8
  joint_angles_pub.publish(msg); 
}





void ElbowRoll(double var1, double var2, double var3, double var4)
{
  naoJointNames.clear();
  naoJointNames.push_back("LElbowRoll");
  naoJointNames.push_back("LHand");
  naoJointNames.push_back("RElbowRoll");
  naoJointNames.push_back("RHand"); 

  naoJointAngles.clear();
  naoJointAngles.push_back(degreesToRadians(var1)); 
  naoJointAngles.push_back(var2);
  naoJointAngles.push_back(degreesToRadians(var3));
  naoJointAngles.push_back(var4);
  speed = 0.1;
  rel = 0;
  msg.joint_names = naoJointNames;
  msg.joint_angles = naoJointAngles;  // float [] -In Radians (must be array)
  msg.speed = speed; // float 
  msg.relative = rel; // unit8
  joint_angles_pub.publish(msg); 
}






void RotarCodosYaw()
{
  
  naoJointNames.clear();
  naoJointNames.push_back("LElbowYaw");
  naoJointNames.push_back("LHand");
  naoJointNames.push_back("RElbowYaw");
  naoJointNames.push_back("RHand"); 

  naoJointAngles.clear();
  naoJointAngles.push_back(degreesToRadians(-10)); 
  naoJointAngles.push_back(1);
  naoJointAngles.push_back(degreesToRadians(10));
  naoJointAngles.push_back(1);
  speed = 0.1;
  rel = 0;
  msg.joint_names = naoJointNames;
  msg.joint_angles = naoJointAngles;  // float [] -In Radians (must be array)
  msg.speed = speed; // float 
  msg.relative = rel; // unit8
  joint_angles_pub.publish(msg); 
}

void RotarMunecas()
{
  
  naoJointNames.clear();
  naoJointNames.push_back("LWristYaw");
  naoJointNames.push_back("LHand");
  naoJointNames.push_back("RWristYaw");
  naoJointNames.push_back("RHand"); 

  naoJointAngles.clear();
  naoJointAngles.push_back(degreesToRadians(-90)); 
  naoJointAngles.push_back(1);
  naoJointAngles.push_back(degreesToRadians(90));
  naoJointAngles.push_back(1);
  speed = 0.1;
  rel = 0;
  msg.joint_names = naoJointNames;
  msg.joint_angles = naoJointAngles;  // float [] -In Radians (must be array)
  msg.speed = speed; // float 
  msg.relative = rel; // unit8
  joint_angles_pub.publish(msg); 
}


void AgarrarBotella()
{
  
  naoJointNames.clear();
  naoJointNames.push_back("LElbowRoll");
  naoJointNames.push_back("LHand");
  naoJointNames.push_back("RElbowRoll");
  naoJointNames.push_back("RHand"); 

  naoJointAngles.clear();
  naoJointAngles.push_back(degreesToRadians(-45)); 
  naoJointAngles.push_back(1);
  naoJointAngles.push_back(degreesToRadians(45));
  naoJointAngles.push_back(1);
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


void DesplazarBaseXYZ(float X, float Y, float Z)
{
  geometry_msgs::PoseStamped MoveBaseSimpleGoalMessage1;
  MoveBaseSimpleGoalMessage1.header.frame_id = "torso";
  MoveBaseSimpleGoalMessage1.pose.position.x=X;
  MoveBaseSimpleGoalMessage1.pose.position.y=Y;
  MoveBaseSimpleGoalMessage1.pose.position.z=Z;
  MoveBaseSimpleGoalMessage1.pose.orientation.w = 1; 
  moveBase_pub.publish(MoveBaseSimpleGoalMessage1);
}








void callBack(const rcnn_live_detector::msgTomarObjeto ReceivedMessage)
{
  std::cout <<"****Estoy en Dejar objeto***" << std::endl;
  
  int contadorImagenes;
  contadorImagenes = 0;
  float DistanciaAvancePorcentaje;
  double tiempoDeEspera;
  bool flagMenorA1Metro;
  flagMenorA1Metro = true;
  float distanciaDelObjeto = 10;//10 metros
  float centroideObjetoRGBX;
  float CorreccionAnguloImage;
  sensor_msgs::ImagePtr CorreccionCVimagePuntero;

  cv::Mat CroppedImageDepth;
  cv::Mat CroppedImageRGB;

  char QRpathacercarseFinoInicio[512];
  char QRpathacercarseFinoDEPTH[512];
  char QRpathacercarseFinoDrawCenteredLine[512];
  char QRpathacercarseFinoDetectedBBOX[512];
  char QRpathCroppedImageDepth[512];
  char QRpathCroppedImageRGB[512];
  std::string MessageFromCallBack = ReceivedMessage.objeto;
  double TopeVirtual = ReceivedMessage.topevirtual;


  float AvanzarHaciaElObjeto;



  rcnn_live_detector::imageTagger       servicioClasificacion;
  sensor_msgs::ImagePtr in_image_RGB;

  in_image_RGB   = punteroImagenRGB();  
  servicioClasificacion.request.image = *in_image_RGB;

  if (clienteClasificacion.call(servicioClasificacion))
  {
    //std::cout << servicioClasificacion.response.tag << std::endl;
    std::cout << "Received information from rcnn_tagger node." << std::endl;

    int n = servicioClasificacion.response.tags.n;
    std::vector<rcnn_live_detector::Prediction> listOfPredictions = servicioClasificacion.response.tags.predictions;
    //declara un vector de objetos tipo Prediction


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


    AvanzarHaciaElObjeto=0.6;

    if(MessageFromCallBack.compare("QRcodeGOOD") == 0)
    {
      ShoulderPitch(-10,1,-10,1);
      sleep(2);
      AvanzarHaciaElObjeto=0.55;       
    }         
    
    if(AvanzarHaciaElObjeto > TopeVirtual)
    {
      AvanzarHaciaElObjeto = TopeVirtual-0.05;
      std::cout<< "ADVERTENCIA: La distancia Suministrada supera el tope Virtual" << std::endl;    
    }
    
    std::cout << "Distancia que avanza el robot: " << AvanzarHaciaElObjeto << std::endl;
    DesplazarBaseXYZ(AvanzarHaciaElObjeto,0,0); 
    sleep(4);







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
      bboxOfObjectDetected.push_back(bbox.at(0)/2);
      bboxOfObjectDetected.push_back(bbox.at(1)/2);
      bboxOfObjectDetected.push_back(bbox.at(2)/2);
      bboxOfObjectDetected.push_back(bbox.at(3)/2);
      std::cout <<"--------Evaluar condicion si corresponde objeto------------" <<std::endl;
      if(tag.compare(MessageFromCallBack) == 0)
      {


        cv::Point p1 = cv::Point(bbox.at(0),bbox.at(1));
        cv::Point p2 = cv::Point(bbox.at(2),bbox.at(3));
        cv::Rect mirectangulo(p1,p2);
        centroideObjetoRGBX=(bbox.at(0)+bbox.at(2))/2;
        
        //CorreccionAnguloImage = -0.1725*centroideObjetoRGBX+27.6;//320x240
        CorreccionAnguloImage = 27.6 - centroideObjetoRGBX*55.2/640;//640x480
        std::cout << "Correccion Angulo Imagen: " << CorreccionAnguloImage << std::endl;
        
        RotateBaseYPR(CorreccionAnguloImage,0,0);
        sleep(1);

        

        
        if(MessageFromCallBack.compare("QRcodeGOOD") == 0)
        {
          ShoulderPitch(5,1,5,1);
          sleep(6);

          ShoulderRoll(50,1,-50,1);
          //system("python /home/steven/importantPythonScripts/soltarobjeto.py"); 
          sleep(6);
                   
          DesplazarBaseXYZ(-0.6,0,0);
          sleep(3);
          system("python /home/steven/importantPythonScripts/restPose.py"); 
          
          system("rostopic pub /speech std_msgs/String \"data: 'He terminado'\"");
          
        }  

        

        CorreccionCVimagePuntero = punteroImagenRGB();
        cv_bridge::CvImageConstPtr CorreccionCVimageBridge = cv_bridge::toCvShare(CorreccionCVimagePuntero, sensor_msgs::image_encodings::BGR8);
        cv::Mat CorreccionCVimageMat = CorreccionCVimageBridge->image;
        snprintf(QRpathacercarseFinoDetectedBBOX, 512, "/home/steven/Desktop/debug/QRTomarObjetoDetectedBBOX%03d.jpg", contadorImagenes); 
        cv::imwrite(QRpathacercarseFinoDetectedBBOX, CorreccionCVimageMat);

        cv::line(CorreccionCVimageMat, cv::Point(320,5), cv::Point(320,315), cv::Scalar(0,0,255), 1, 8);
        snprintf(QRpathacercarseFinoDrawCenteredLine, 512, "/home/steven/Desktop/debug/QRTomarObjetoDrawCenteredLine%03d.jpg", contadorImagenes); 
        cv::imwrite(QRpathacercarseFinoDrawCenteredLine, CorreccionCVimageMat);

          

       }

    }

       

    

    std::cout << ss.str();

    std_msgs::String msg;
    msg.data = ss.str();



    }
    else
    {
    ROS_ERROR("Failed to call service tag_Service");
    //return 1;
    }


}






int main(int argc, char **argv)
{
  system("python /home/steven/importantPythonScripts/blockArmsWhenWalk.py");


  ros::init(argc, argv, "DejarObjeto");
  ROS_INFO("inicio de nodo DejarObjeto");
  nh_ptr_ = boost::make_shared<ros::NodeHandle>();

  //Cliente de servicio de clasificacion
  clienteClasificacion = nh_ptr_->serviceClient<rcnn_live_detector::imageTagger>("/qr_service_project_panda");
  clienteProfundidad   = nh_ptr_->serviceClient<rcnn_live_detector::projectPandaProfundidadServer>("/depth_service_project_panda");





  //Suscribirse al reconocimiento de voz
  ReceiveMsgFromAcercarseFino = nh_ptr_->subscribe<rcnn_live_detector::msgTomarObjeto>("goDropTheObject", 1000, callBack);



  

  //Publicar a mover base 
  moveBase_pub = nh_ptr_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);



  //publicar a las articulaciones de la cabeza
  joint_angles_pub = nh_ptr_->advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("/joint_angles" ,1);


  

  /*******Declarar nombres de articulaciones******/
  /*
  naoJointNames.clear();
  naoJointNames.push_back("LShoulderPitch");
  naoJointNames.push_back("LShoulderRoll");
  naoJointNames.push_back("LElbowYaw");
  naoJointNames.push_back("LElbowRoll");
  naoJointNames.push_back("LWristYaw");
  naoJointNames.push_back("LHand");
  naoJointNames.push_back("RShoulderPitch");
  naoJointNames.push_back("RShoulderRoll");
  naoJointNames.push_back("RElbowYaw");
  naoJointNames.push_back("RElbowRoll");
  naoJointNames.push_back("RWristYaw");
  naoJointNames.push_back("RHand"); 
  */ 
  /*****************************************/

  ros::spin();

  return 0;

}