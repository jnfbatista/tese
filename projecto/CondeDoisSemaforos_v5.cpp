#include <stdio.h>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include "Message/Message.h"
#include "Message/Message.cpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//Versão opencv 2.1
//#include <cv.h>
//#include <highgui.h>
//#include <imgproc.h>

/// TODO
//: adicionar vector de Mat's
//vector<Mat> input[5];
//input[0] = imread(filename, type);
/// TODO

using namespace cv;
using namespace std;

/// Declaração Funções
void leitura_templates();
void envio_mensagem(string parametro);
string matching_templates(Size r0, Mat &image, Point2f center);

/// Declaração variaveis
/// Grau de confiança para matching
float nivel_confianca = 0.8;
/// Nivel do threshold
int nivel_threshold = 80;
/// Numero de templates
int numero_templates = 30;
/// Definição do IP e PORT
//string ip_adress = "192.168.43.216";
//string ip_adress = "192.168.43.13";
string ip_adress = "127.0.0.1"; /// localhost
//string ip_adress = argv[2];
string port = "4447";
/// Escolha dos templates para verificação
bool ativar_template0 = true; // template original: Semáforo Frente
bool ativar_template1 = true; // template original: Semáforo Direita
bool ativar_template2 = true; // template original: Semáforo Esquerda
bool ativar_template3 = true; // template original: Semáforo Parar
bool ativar_template4 = true; // template original: Semáforo Fim Manga
bool ativar_template5 = true; // template original: Sinal Info Velocidade
bool ativar_template6 = true; // template original: Sinal Info Hospital
bool ativar_template7 = true; // template original: Sinal Mandatório Bus
bool ativar_template8 = true; // template original: Sinal Mandatório Luzes
bool ativar_template9 = true; // template original: Sinal Perigo Estrada
bool ativar_template10 = true; // template original: Sinal Perigo Lomba
bool ativar_template11 = false; // template do local da competição: Semáforo Frente
bool ativar_template12 = false; // template do local da competição: Semáforo Direita
bool ativar_template13 = false; // template do local da competição: Semáforo Esquerda
bool ativar_template14 = false; // template do local da competição: Semáforo Parar
bool ativar_template15 = false; // template do local da competição: Semáforo Fim Manga
bool ativar_template16 = false; // template do local da competição: Sinal Info Velocidade
bool ativar_template17 = false; // template do local da competição: Sinal Info Hospital
bool ativar_template18 = false; // template do local da competição: Sinal Mandatório Bus
bool ativar_template19 = false; // template do local da competição: Sinal Mandatório Luzes
bool ativar_template20 = false; // template do local da competição: Sinal Perigo Estrada
bool ativar_template21 = false; // template do local da competição: Sinal Perigo Lomba
bool ativar_template22 = false; // extra 1 ...
bool ativar_template23 = false; // extra 2 ...
bool ativar_template24 = false; // extra 3 ...
bool ativar_template25 = false; // extra 4 ...
bool ativar_template26 = false; // extra 5 ...
bool ativar_template27 = false; // extra 6 ...
bool ativar_template28 = false; // extra 7 ...
bool ativar_template29 = false; // extra 8 ...

//! templates dos semaforos
Mat tpl_frente, tpl_esquerda, tpl_direita, tpl_parar, tpl_fimmanga;
Mat tpl_frente_debug, tpl_esquerda_debug, tpl_direita_debug, tpl_parar_debug, tpl_fimmanga_debug;
//! templates dos sinais
Mat tpl_velocidade, tpl_hospital, tpl_bus, tpl_luzes, tpl_estrada, tpl_lomba;
Mat tpl_velocidade_debug, tpl_hospital_debug, tpl_bus_debug, tpl_luzes_debug, tpl_estrada_debug, tpl_lomba_debug;
Mat tpl_extra1, tpl_extra2, tpl_extra3, tpl_extra4, tpl_extra5, tpl_extra6, tpl_extra7, tpl_extra8;

int main( int argc, char** argv )
{
    //! Abrir video
    VideoCapture cap = VideoCapture(atoi(argv[1]));
    //! Verificar abertura do video
    if(!cap.isOpened())
    {
        std::cout << "Não conseguiu abrir video!" << std::endl;
        //! Leitura de imagem
        Mat frame = imread(argv[1], CV_LOAD_IMAGE_COLOR);
        //! Se frame vazia, termina
        if (frame.empty())
        {
            std::cout << "Não conseguiu abrir imagem. Introduza argumentos correctos!" << std::endl;
            return 0;
        }
        //! Senão faz o processamento da imagem
    }
    //! Calibração da câmara
    else {
        cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
        //cap.set(CV_CAP_PROP_CONTRAST,0.8);
        //cap.set(CV_CAP_PROP_BRIGHTNESS,0.7);
    }
    //! Inicialização do algoritmo
    int img_width,img_height,tpl_width,tpl_height,res_width,res_height;
    Mat frame, gray, canny, contornos;
    int key=0;
    //! Variavel para deteção dos contornos
    vector<vector<Point> > linhas;

    //! Leitura dos templates
    leitura_templates();

    //! Preparação da apresentação das janelas
    namedWindow("Video",CV_WINDOW_AUTOSIZE);
    //namedWindow("Gray",CV_WINDOW_AUTOSIZE);
    //namedWindow("Contornos",CV_WINDOW_AUTOSIZE);
    namedWindow("Canny",CV_WINDOW_AUTOSIZE);
    //! Obter frame seguinte da camara
    cap >> frame;
    for(;;)
    {
        //! teclar letra q para sair da aplicação
        while(key != 'q')
        {
            //! Processamento do video
            cap >> frame;
            /// Aplicar filtro pryDown
            pyrDown(frame,frame);
            //! Segmentação
            cvtColor(frame,gray,CV_BGR2GRAY);
            cvtColor(frame,canny,CV_BGR2GRAY);

            //! Método 1 - Threshold (Gray), FindContours, Limita area Rect, Resize template, Matching
            //threshold(gray,gray,nivel_threshold,255,THRESH_BINARY);


            //! Método 2 - Canny, FindContours, Limita area Rect, Resize template, Matching
            Canny(canny,canny,nivel_threshold,200,3);


            /*
            //! Método 3 - Threshold (cores), FindContours, Limita area Rect, Resize template, Matching
            threshold(frame,binary,nivel_threshold,255,THRESH_BINARY);
            threshold(gray,gray,128,255,THRESH_BINARY_INV);
            threshold(gray,gray,255,0,THRESH_BINARY);
            */


            contornos = canny.clone();
            findContours(contornos,linhas,RETR_LIST,CHAIN_APPROX_NONE);

            //! iterate over all contours
            for (uint k=0; k<linhas.size();k++)
            {
                int areaContorno = contourArea(linhas[k]);
                //640x480 = 1000<x<4000  <- without pyrdown
                //320x240 = 100<x<2000   <- with pyrdown
                //! Analisa apenas rectangulos com area de contorno entre minimo a máximo
                if (areaContorno > 200 && areaContorno < 2000)
                {
                    /// bouding rect - desenho dos rectangulos
                    Rect r0 = boundingRect(linhas[k]);
                    rectangle(canny,r0,Scalar(255));
                    //rectangle(contornos,r0,Scalar(255));
                    //rectangle(frame,r0,Scalar(0,0,255));
                    //! compute all moments
                    Moments mom= moments(Mat(linhas[k]));
                    //! draw mass center - draw white dot
                    //circle(gray,Point(mom.m10/mom.m00,mom.m01/mom.m00),2,Scalar(255),2);
                    imshow("Video",frame);
                    string num = matching_templates(Size(r0.width,r0.height),frame,Point2f(mom.m10/mom.m00,mom.m01/mom.m00));
                    if (num!="") {
                        envio_mensagem(num);
                        num = "";
                    }
                }
            }
            //imshow("Binario",binary);
            //imshow("Contornos",contornos);
            imshow("Canny",canny);
            //imshow("Gray",gray);
            key = waitKey(1);

        }
        break;
    }
    return 0;
}

//! Leitura templates dos semaforos e sinais
void leitura_templates()
{
    if (ativar_template0) tpl_frente = imread("Semaforo_Frente.jpg",CV_LOAD_IMAGE_COLOR); //0
    if (ativar_template1) tpl_direita = imread("Semaforo_Direita.jpg",CV_LOAD_IMAGE_COLOR); //1
    if (ativar_template2) tpl_esquerda = imread("Semaforo_Esquerda.jpg",CV_LOAD_IMAGE_COLOR); //2
    if (ativar_template3) tpl_parar = imread("Semaforo_Parar.jpg",CV_LOAD_IMAGE_COLOR); //3
    if (ativar_template4) tpl_fimmanga = imread("Semaforo_FimManga.jpg",CV_LOAD_IMAGE_COLOR); //4
    if (ativar_template5) tpl_velocidade = imread("Sinal_Velocidade.jpg",CV_LOAD_IMAGE_COLOR); //5
    if (ativar_template6) tpl_hospital = imread("Sinal_Hospital.jpg",CV_LOAD_IMAGE_COLOR); //6
    if (ativar_template7) tpl_bus = imread("Sinal_Bus.jpg",CV_LOAD_IMAGE_COLOR); //7
    if (ativar_template8) tpl_luzes = imread("Sinal_Luzes.jpg",CV_LOAD_IMAGE_COLOR); //8
    if (ativar_template9) tpl_estrada = imread("Sinal_Estrada.jpg",CV_LOAD_IMAGE_COLOR); //9
    if (ativar_template10) tpl_lomba = imread("Sinal_Lomba.jpg",CV_LOAD_IMAGE_COLOR); //10
    if (ativar_template11) tpl_frente_debug = imread("Semaforo_Frente_Debug.jpg",CV_LOAD_IMAGE_COLOR); //11
    if (ativar_template12) tpl_direita_debug = imread("Semaforo_Direita_Debug.jpg",CV_LOAD_IMAGE_COLOR); //12
    if (ativar_template13) tpl_esquerda_debug = imread("Semaforo_Esquerda_Debug.jpg",CV_LOAD_IMAGE_COLOR); //13
    if (ativar_template14) tpl_parar_debug = imread("Semaforo_Parar_Debug.jpg",CV_LOAD_IMAGE_COLOR); //14
    if (ativar_template15) tpl_fimmanga_debug = imread("Semaforo_FimManga_Debug.jpg",CV_LOAD_IMAGE_COLOR); //15
    if (ativar_template16) tpl_velocidade_debug = imread("Sinal_Velocidade_Debug.jpg",CV_LOAD_IMAGE_COLOR); //16
    if (ativar_template17) tpl_hospital_debug = imread("Sinal_Hospital_Debug.jpg",CV_LOAD_IMAGE_COLOR); //17
    if (ativar_template18) tpl_bus_debug = imread("Sinal_Bus_Debug.jpg",CV_LOAD_IMAGE_COLOR); //18
    if (ativar_template19) tpl_luzes_debug = imread("Sinal_Luzes_Debug.jpg",CV_LOAD_IMAGE_COLOR); //19
    if (ativar_template20) tpl_estrada_debug = imread("Sinal_Estrada_Debug.jpg",CV_LOAD_IMAGE_COLOR); //20
    if (ativar_template21) tpl_lomba_debug = imread("Sinal_Lomba_Debug.jpg",CV_LOAD_IMAGE_COLOR); //21
    if (ativar_template22) tpl_extra1 = imread("Extra1.jpg",CV_LOAD_IMAGE_COLOR); //22
    if (ativar_template23) tpl_extra2 = imread("Extra2.jpg",CV_LOAD_IMAGE_COLOR); //23
    if (ativar_template24) tpl_extra3 = imread("Extra3.jpg",CV_LOAD_IMAGE_COLOR); //24
    if (ativar_template25) tpl_extra4 = imread("Extra4.jpg",CV_LOAD_IMAGE_COLOR); //25
    if (ativar_template26) tpl_extra5 = imread("Extra5.jpg",CV_LOAD_IMAGE_COLOR); //26
    if (ativar_template27) tpl_extra6 = imread("Extra6.jpg",CV_LOAD_IMAGE_COLOR); //27
    if (ativar_template28) tpl_extra7 = imread("Extra7.jpg",CV_LOAD_IMAGE_COLOR); //28
    if (ativar_template29) tpl_extra8 = imread("Extra8.jpg",CV_LOAD_IMAGE_COLOR); //29
}

//! Envio da Mensagem
void envio_mensagem(string parametro) {
    Message msg;
    msg.add_param("semaforo",parametro);
    msg.send_message(ip_adress,port);
}

//! Analisar Matching
string matching_templates(Size r0, Mat &image, Point2f center) {
    Mat template1, recorteimage, resultado;
    float matriz[numero_templates], maximo;
    int posicao;
    string text;
    //! Recorta imagem - Region of Interrest
    getRectSubPix(image,Size(r0.width,r0.height),center,recorteimage);
    for(int k=0;k<numero_templates;k++) {
        //! Redimensionar template para tamanha do rectangulo encontrado
        if (k==0 && ativar_template0){ resize(tpl_frente,template1,Size(r0.width,r0.height));
        }
        if (k==1 && ativar_template1){ resize(tpl_direita,template1,Size(r0.width,r0.height));
        }
        if (k==2 && ativar_template2){ resize(tpl_esquerda,template1,Size(r0.width,r0.height));
        }
        if (k==3 && ativar_template3){ resize(tpl_parar,template1,Size(r0.width,r0.height));
        }
        if (k==4 && ativar_template4){ resize(tpl_fimmanga,template1,Size(r0.width,r0.height));
        }
        if (k==5 && ativar_template5){ resize(tpl_velocidade,template1,Size(r0.width,r0.height));
        }
        if (k==6 && ativar_template6){ resize(tpl_hospital,template1,Size(r0.width,r0.height));
        }
        if (k==7 && ativar_template7){ resize(tpl_bus,template1,Size(r0.width,r0.height));
        }
        if (k==8 && ativar_template8){ resize(tpl_luzes,template1,Size(r0.width,r0.height));
        }
        if (k==9 && ativar_template9){ resize(tpl_estrada,template1,Size(r0.width,r0.height));
        }
        if (k==10 && ativar_template10){ resize(tpl_lomba,template1,Size(r0.width,r0.height));
        }
        if (k==11 && ativar_template11){ resize(tpl_frente_debug,template1,Size(r0.width,r0.height));
        }
        if (k==12 && ativar_template12){ resize(tpl_direita_debug,template1,Size(r0.width,r0.height));
        }
        if (k==13 && ativar_template13){ resize(tpl_esquerda_debug,template1,Size(r0.width,r0.height));
        }
        if (k==14 && ativar_template14){ resize(tpl_parar_debug,template1,Size(r0.width,r0.height));
        }
        if (k==15 && ativar_template15){ resize(tpl_fimmanga_debug,template1,Size(r0.width,r0.height));
        }
        if (k==16 && ativar_template16){ resize(tpl_velocidade_debug,template1,Size(r0.width,r0.height));
        }
        if (k==17 && ativar_template17){ resize(tpl_hospital_debug,template1,Size(r0.width,r0.height));
        }
        if (k==18 && ativar_template18){ resize(tpl_bus_debug,template1,Size(r0.width,r0.height));
        }
        if (k==19 && ativar_template19){ resize(tpl_luzes_debug,template1,Size(r0.width,r0.height));
        }
        if (k==20 && ativar_template20){ resize(tpl_estrada_debug,template1,Size(r0.width,r0.height));
        }
        if (k==21 && ativar_template21){ resize(tpl_lomba_debug,template1,Size(r0.width,r0.height));
        }
        if (k==22 && ativar_template22){ resize(tpl_extra1,template1,Size(r0.width,r0.height));
        }
        if (k==23 && ativar_template23){ resize(tpl_extra2,template1,Size(r0.width,r0.height));
        }
        if (k==24 && ativar_template24){ resize(tpl_extra3,template1,Size(r0.width,r0.height));
        }
        if (k==25 && ativar_template25){ resize(tpl_extra4,template1,Size(r0.width,r0.height));
        }
        if (k==26 && ativar_template26){ resize(tpl_extra5,template1,Size(r0.width,r0.height));
        }
        if (k==27 && ativar_template27){ resize(tpl_extra6,template1,Size(r0.width,r0.height));
        }
        if (k==28 && ativar_template28){ resize(tpl_extra7,template1,Size(r0.width,r0.height));
        }
        if (k==29 && ativar_template29){ resize(tpl_extra8,template1,Size(r0.width,r0.height));
        }
        //! Compara template com imagem recortada
        matchTemplate(recorteimage,template1,resultado,CV_TM_CCORR_NORMED);
        //if (k==9) cout << resultado.at<float>(0,0) << endl;
        matriz[k]=resultado.at<float>(0,0);
    }
    //cout << "fim contorno" << endl;
    //waitKey();
    maximo = *max_element(matriz,matriz+numero_templates);
    posicao = distance(matriz,max_element(matriz,matriz+numero_templates));
    if(maximo>nivel_confianca) {
        imshow("recorte da imagem",recorteimage);
        if (posicao==0 && ativar_template0) {
            imshow("Template",tpl_frente);
            text = "frente";
        }
        if (posicao==1 && ativar_template1) {
            imshow("Template",tpl_direita);
            text = "direita";
        }
        if (posicao==2 && ativar_template2) {
            imshow("Template",tpl_esquerda);
            text = "esquerda";
        }
        if (posicao==3 && ativar_template3) {
            imshow("Template",tpl_parar);
            text = "parar";
        }
        if (posicao==4 && ativar_template4) {
            imshow("Template",tpl_fimmanga);
            text = "fimmanga";
        }
        if (posicao==5 && ativar_template5) {
            imshow("Template",tpl_velocidade);
            text = "velocidade";
        }
        if (posicao==6 && ativar_template6) {
            imshow("Template",tpl_hospital);
            text = "hospital";
        }
        if (posicao==7 && ativar_template7) {
            imshow("Template",tpl_bus);
            text = "bus";
        }
        if (posicao==8 && ativar_template8) {
            imshow("Template",tpl_luzes);
            text = "luzes";
        }
        if (posicao==9 && ativar_template9) {
            imshow("Template",tpl_estrada);
            text = "estrada";
        }
        if (posicao==10 && ativar_template10) {
            imshow("Template",tpl_lomba);
            text = "lomba";
        }
        if (posicao==11 && ativar_template11) {
            imshow("Template",tpl_frente_debug);
            text = "frente";
        }
        if (posicao==12 && ativar_template12) {
            imshow("Template",tpl_direita_debug);
            text = "direita";
        }
        if (posicao==13 && ativar_template13) {
            imshow("Template",tpl_esquerda_debug);
            text = "esquerda";
        }
        if (posicao==14 && ativar_template14) {
            imshow("Template",tpl_parar_debug);
            text = "parar";
        }
        if (posicao==15 && ativar_template15) {
            imshow("Template",tpl_fimmanga_debug);
            text = "fimmanga";
        }
        if (posicao==16 && ativar_template16) {
            imshow("Template",tpl_velocidade_debug);
            text = "velocidade";
        }
        if (posicao==17 && ativar_template17) {
            imshow("Template",tpl_hospital_debug);
            text = "hospital";
        }
        if (posicao==18 && ativar_template18) {
            imshow("Template",tpl_bus_debug);
            text = "bus";
        }
        if (posicao==19 && ativar_template19) {
            imshow("Template",tpl_luzes_debug);
            text = "luzes";
        }
        if (posicao==20 && ativar_template20) {
            imshow("Template",tpl_estrada_debug);
            text = "estrada";
        }
        if (posicao==21 && ativar_template21) {
            imshow("Template",tpl_lomba_debug);
            text = "lomba";
        }
        if (posicao==22 && ativar_template22) {
            imshow("Template",tpl_extra1);
            text = "extra1";
        }
        if (posicao==23 && ativar_template23) {
            imshow("Template",tpl_extra2);
            text = "extra2";
        }
        if (posicao==24 && ativar_template24) {
            imshow("Template",tpl_extra3);
            text = "extra3";
        }
        if (posicao==25 && ativar_template25) {
            imshow("Template",tpl_extra4);
            text = "extra4";
        }
        if (posicao==26 && ativar_template26) {
            imshow("Template",tpl_extra5);
            text = "extra5";
        }
        if (posicao==27 && ativar_template27) {
            imshow("Template",tpl_extra6);
            text = "extra6";
        }
        if (posicao==28 && ativar_template28) {
            imshow("Template",tpl_extra7);
            text = "extra7";
        }
        if (posicao==29 && ativar_template29) {
            imshow("Template",tpl_extra8);
            text = "extra8";
        }
        cout << maximo << endl;
    }
    //cout << posicao << endl;
    return text;
}

