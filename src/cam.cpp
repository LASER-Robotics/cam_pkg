//Bibliotecas necessarias

#include <ros/ros.h> //Biblioteca do ROS
#include <image_transport/image_transport.h> //Biblioteca da captura de imagens
#include <opencv2/highgui/highgui.hpp> //Biblioteca da captura de imagens
#include <opencv2/core.hpp> ///Biblioteca da captura de imagens
#include <opencv2/imgproc/imgproc.hpp> //Biblioteca da captura de imagens
#include <cv_bridge/cv_bridge.h> //Biblioteca da captura de imagens
#include <vector> // Biblioteca para criação de vetores

// Guarda as imagens enviadas pelo topico da camera
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // Lê a imagem
    cv::Mat gray = cv_bridge::toCvShare(msg, "bgr8")->image;
    // Verifica se a imagem esta sendo transmitida
    if( gray.empty() )
     {
       ROS_INFO("Could not open or find the image");
       ros::shutdown();
     }
    // Abre uma janela que transmite a imagem que esta sendo recebida
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);

  }
  // Caso ocorra erro na captura da imagem retorna um alerta
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

// Função principal que inicializa o algoritmo
int main(int argc, char **argv)
{
  // Inicializa o ROS
  ros::init(argc, argv, "image_listener");
  // Declara o node que vai ser atrelado
  ros::NodeHandle nh;
  // Inicia a janela de transmissão da imagem
  cv::namedWindow("view");

  // Busca o topico especificado que publica a imagem da camera
  image_transport::ImageTransport it(nh);
  // Subscriber que recebe a imagem e inicia a função imageCallback
  image_transport::Subscriber sub = it.subscribe("/my_robot/camera1/image_raw", 1, imageCallback); 
  // Finaliza a janela da transmissão
  cv::destroyWindow("view");

  ros::spin();
}
