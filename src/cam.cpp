//Bibliotecas necessarias

#include <ros/ros.h> //Biblioteca do ROS
#include <image_transport/image_transport.h> //Biblioteca da captura de imagens
#include <opencv2/highgui/highgui.hpp> //Biblioteca da captura de imagens
#include <opencv2/core.hpp> ///Biblioteca da captura de imagens
#include <opencv2/imgproc/imgproc.hpp> //Biblioteca da captura de imagens
#include <cv_bridge/cv_bridge.h> //Biblioteca da captura de imagens
#include <vector> // Biblioteca para criação de vetores
#include <zbar.h> // Biblioteca para decodificação de codigos de barras e QRcodes

// Declaração da estrutura dos dados fornecidos pelos codigos de barras ou QRcodes
typedef struct
{
  std::string type;
  std::string data;
  std::vector <cv::Point> location;
}decodedObject;

// Procura na imagem o codigo e decodifica (FUNÇÂO DA BIBLIOTECA ZBAR)
void decode(cv::Mat &im, std::vector<decodedObject>&decodedObjects)
{

  // Cria o scanner da biblioteca zbar
  zbar::ImageScanner scanner;

  // Configura o scanner
  scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

  // Converte a imagem para a cor cinza
  cv::Mat imGray;
  cv::cvtColor(im, imGray,CV_BGR2GRAY);

  // Recorta a area a imagem
  zbar::Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);

  // Scaneia a imagem em busca do codigo
  int n = scanner.scan(image);

  // Mostra os resultados do escaneamento
  for(zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
  {
    decodedObject obj;

    obj.type = symbol->get_type_name();
    obj.data = symbol->get_data();

    // Printa no terminal os dados encontrados e o tipo de codigo lido
    std::cout << "Type : " << obj.type << std::endl;
    std::cout << "Data : " << obj.data << std::endl << std::endl;
    decodedObjects.push_back(obj);
  }
}

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
    // Declaração da variavel que decodifica os codigos
    std::vector<decodedObject> decodedObjects;
    // Inicia a função de decodificação
    decode(gray, decodedObjects);

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
  image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback); 
  // Finaliza a janela da transmissão
  cv::destroyWindow("view");

  ros::spin();
}
