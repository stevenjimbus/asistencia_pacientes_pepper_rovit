#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include "rcnn_live_detector/Prediction.h"
#include "rcnn_live_detector/PredictionsList.h"
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
#include <math.h>  

//NODOS
ros::ServiceServer service;

//Manejo de nodos para ROS
boost::shared_ptr<ros::NodeHandle> nh_ptr_;

//Paths
//const std::string RECEIVE_IMG_TOPIC_NAME = "/camera/depth/image_rect_raw";


bool calcularProfundidad(profundidad_service::profundidadServer::Request &req,
             profundidad_service::profundidadServer::Response &res)
{



  /*Se lee los datos del request*/
  std::vector<rcnn_live_detector::Prediction> listOfPredictions = req.tags.predictions;

  cv_bridge::CvImagePtr depth = cv_bridge::toCvCopy(req.image , sensor_msgs::image_encodings::TYPE_32FC1);
  std::vector<profundidad_service::distance_msg> ListaDeDistanciaYPrediction;/*se genera la lista de 
                                                                              de response del servicio
                                                                               en base al profundidadServer */
  /*Se procesa la información que se desea enviar*/

  cv::Mat imagenMatriz =depth->image;
  cv::Mat CroppedImage;


  int CroppedImageWidth;
  int CroppedImageHeight;
  float SumaDeProfundidades = 0.0f;
  float DistanciaPromedio =  0.0f;
  
  float ProfundidadDePixel =  0.0f;

  
      

 
  for (int i = 0; i < listOfPredictions.size(); ++i)
    { 
        //bbox.at(3)
      //Declarar el paquete que envuelve la distancia y la prediccion del objeto reconocido
      profundidad_service::distance_msg respuesta_de_distancia;/*respuesta_de_distancia es lo que voy a enviar 
                                                                 por el response de profundidadServer como un 
                                                                 vector de distance_msg */
      
      std::vector<double> bbox = listOfPredictions.at(i).bbox;
      cv::Point p1 = cv::Point(bbox.at(0),bbox.at(1));
      cv::Point p2 = cv::Point(bbox.at(2),bbox.at(3));
      

      cv::Rect mirectangulo(p1,p2);
      CroppedImage=imagenMatriz(mirectangulo);

      CroppedImageWidth=CroppedImage.size().width; 
      CroppedImageHeight=CroppedImage.size().height;
      /*
      std::cout <<"Width de la imagen"<< CroppedImageWidth << std::endl;
      std::cout <<"Length de la imagen"<< CroppedImageHeight<< std::endl;
      */
      int CantidadDePixeles=0;
      int contador=0;
      for (int j=0;j<CroppedImageWidth;++j){
        for (int k=0;k<CroppedImageHeight;++k){  
          ++CantidadDePixeles;        
          ProfundidadDePixel=CroppedImage.at<float> (j,k);          
          //SumaDeProfundidades = SumaDeProfundidades + CroppedImage.at<float>(CroppedImageWidth,CroppedImageHeight);
          
          if (!isnan(ProfundidadDePixel)){
            ++contador;
            SumaDeProfundidades=ProfundidadDePixel+SumaDeProfundidades;
          }

          
        }
      }
      float eficiencia= 0.0f;
      eficiencia=100*(((float)contador)/((float)CantidadDePixeles));
      /*
      std::cout << "Cantidad elementos validos" << contador << std::endl;      
      std::cout << "Cantidad pixeles" << CantidadDePixeles << std::endl;
      std::cout << "Eficiencia segun la cantidad de pixeles validos %f" << eficiencia << std::endl;
      */

      DistanciaPromedio = SumaDeProfundidades/contador;
      //float prof1=CroppedImage.at<float> (0,0);
      //float prof2=imagenMatriz.at<float> (p2);
      //float promprof = ((prof1+prof2)/2);
      //float promprof =prof1;
      float promprof = DistanciaPromedio;
      




      respuesta_de_distancia.distance=promprof;
      respuesta_de_distancia.prediction=listOfPredictions.at(i);

      ListaDeDistanciaYPrediction.push_back(respuesta_de_distancia);
    }



    

    
   

    sensor_msgs::ImagePtr msg_new = cv_bridge::CvImage(std_msgs::Header(), "32FC1",CroppedImage).toImageMsg();
    
    res.imageRespuestaDeServicio=*msg_new;
    res.distanceslist=ListaDeDistanciaYPrediction;

    return true;


  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "servicio_depth");
  ROS_INFO("Inicio del Nodo: servicio_depth");  
  

  nh_ptr_ = boost::make_shared<ros::NodeHandle>();//asignacion memoria dinamica boost



  
  service = nh_ptr_->advertiseService("depth_Service", calcularProfundidad);


  ros::spin();

  return 0;
}