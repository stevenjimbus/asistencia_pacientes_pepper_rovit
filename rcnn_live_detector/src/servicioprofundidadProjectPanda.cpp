#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include "rcnn_live_detector/Prediction.h"
#include "rcnn_live_detector/PredictionsList.h"
//#include "profundidad_service/profundidadServer.h"
//#include "profundidad_service/distance_msg.h"
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
#include <math.h>  

//NODOS
ros::ServiceServer service;

//Manejo de nodos para ROS
boost::shared_ptr<ros::NodeHandle> nh_ptr_;

//Paths
//const std::string RECEIVE_IMG_TOPIC_NAME = "/camera/depth/image_rect_raw";


bool calcularProfundidad(rcnn_live_detector::projectPandaProfundidadServer::Request &req,
             rcnn_live_detector::projectPandaProfundidadServer::Response &res)
{
  std::cout << "*******Llamado a Servicio de Profundidad" << std::endl;
  sensor_msgs::Image imageFromRequest;
  imageFromRequest = req.image;
  cv_bridge::CvImagePtr depth = cv_bridge::toCvCopy(imageFromRequest , sensor_msgs::image_encodings::TYPE_32FC1);
  cv::Mat imagenMatriz =depth->image;
  cv::Mat CroppedImage;


  std::vector<double> bbox;
  bbox = req.bbox;
  double X1 = bbox.at(0);
  double Y1 = bbox.at(1);
  double X2 = bbox.at(2);
  double Y2 = bbox.at(3);

  std::cout << X1 << std::endl;
  std::cout << Y1 << std::endl;
  std::cout << X2 << std::endl;
  std::cout << Y2 << std::endl;

  int CroppedImageWidth;
  int CroppedImageHeight;
  float SumaDeProfundidades = 0.0;
  float DistanciaPromedio =  0.0;  
  float ProfundidadDePixel =  0.0;

  cv::Point p1 = cv::Point(X1,Y1);
  cv::Point p2 = cv::Point(X2,Y2);

  cv::Rect mirectangulo(p1,p2);
  CroppedImage=imagenMatriz(mirectangulo);

  CroppedImageWidth=CroppedImage.size().width; 
  CroppedImageHeight=CroppedImage.size().height;


  int threshold;    
  float step;
  float distancia;
  int ciclos;
  int posicionEnHistograma;
  float DistanciaAlObjeto;
  int contador;
  contador=0;

  step=0.1;
  distancia=3.9;
  ciclos=(distancia/step)+1;
  std::cout << "ciclos: "<<ciclos << std::endl;

  threshold=0;
  



  int ListaFrecuenciaPuntos [ciclos];
  float sumaDeDistancias [ciclos];

  for(int a = 0; a < ciclos; a = a + 1){
    ListaFrecuenciaPuntos[a]=0;
    sumaDeDistancias[a]=0;
  }



  
  for (int j=0;j<CroppedImageWidth;++j){
    for (int k=0;k<CroppedImageHeight;++k){
      contador++;  
      ProfundidadDePixel=CroppedImage.at<float> (j,k);  
      if (!isnan(ProfundidadDePixel))
      {
        posicionEnHistograma= ProfundidadDePixel/step;
        ListaFrecuenciaPuntos[posicionEnHistograma]=ListaFrecuenciaPuntos[posicionEnHistograma]+1; 
        sumaDeDistancias[posicionEnHistograma]=sumaDeDistancias[posicionEnHistograma]+ProfundidadDePixel; 
      }   
    }
  }
  std::cout << "CroppedImageWidth "<< CroppedImageWidth <<std::endl;
  std::cout << "CroppedImageHeight "<< CroppedImageHeight << std::endl;
  std::cout << "Cantidad de pixeles"<< contador<< std::endl;

 

  
  std::cout <<"Total de puntos" << contador << std::endl;
  std::cout << "!!!!!!Histograma!!!!!!" << std::endl;
  std::cout << "Cada * significa 100 puntos" << std::endl;
  for(int b = 0; b < ciclos; b = b + 1){
    printf("Rango [%.2f" , b*step );
    printf("] - [%.2f]   " , (b+1)*step);
    for(int delta=0; delta < ListaFrecuenciaPuntos[b]/100;delta=delta+1)
    {
      std::cout<<"*";
    }
    std::cout<< ListaFrecuenciaPuntos[b]<< std::endl;
  }

  /*
  for(int alpha=0; alpha < ciclos; alpha = alpha + 1)
  {
    std::cout << "sumaDeDistancias[" << alpha << "] = " << sumaDeDistancias[alpha] << std::endl;
  }
  */
  

  float sumaFinalDeDistancias=0;
  float sumaFinalDePuntos=0;
  bool condicionParaSumar = false;
  threshold=contador*0.065;
  std::cout << "threshold: "<<threshold << std::endl;
  for(int c = 0; c < ciclos; c = c + 1){
    //std::cout << "c = " << c << std::endl;
    if(ListaFrecuenciaPuntos[c] > threshold)
    {
      condicionParaSumar = true;
    }



    if(condicionParaSumar)
    {
      
      sumaFinalDeDistancias = sumaDeDistancias[c] + sumaFinalDeDistancias;
      sumaFinalDePuntos = ListaFrecuenciaPuntos[c] + sumaFinalDePuntos;
      if(ListaFrecuenciaPuntos[c] < threshold)
      {
        c=ciclos + 20;
      }

    }
  }




  std::cout<< "sumaFinalDeDistancias" <<  sumaFinalDeDistancias << std::endl;
  std::cout<< "sumaFinalDePuntos" <<  sumaFinalDePuntos << std::endl;

  DistanciaAlObjeto=sumaFinalDeDistancias/sumaFinalDePuntos;
  std::cout<< "Distancia Al Objeto" <<  DistanciaAlObjeto << std::endl;
  


  double promprof = DistanciaAlObjeto;





  res.distance_to_object = promprof;
  return true;


  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "servicioprofundidadProjectPanda");
  ROS_INFO("Inicio del Nodo: servicioprofundidadProjectPanda");  
  

  nh_ptr_ = boost::make_shared<ros::NodeHandle>();//asignacion memoria dinamica boost



  
  service = nh_ptr_->advertiseService("/depth_service_project_panda", calcularProfundidad);




  ros::spin();

  return 0;
}