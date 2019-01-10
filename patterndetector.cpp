#include "patterndetector.h"
#include "mainwindow.h"
#include "image.h"
#include "constants.h"
#include "metrics.h"
#include "geometria.h"
#include <algorithm>
#include <stack>
#include <fstream>
#include <set>
#include <omp.h>

// Convertir floats a strings
template<typename Te>
std::string num2str(Te x) { std::stringstream ss; ss << x; return ss.str();}

cv::RotatedRect RectanguloR;
int indice= 1;
bool mayorpuntosX(cv::Point2f i, cv::Point2f j)
{
    cv::Point2f vertices[4];
    RectanguloR.points(vertices);
    i = i - vertices[indice];
    j = j - vertices[indice];
    return (cos(RectanguloR.angle)*i.x + sin(RectanguloR.angle)*i.y) < (cos(RectanguloR.angle)*j.x + sin(RectanguloR.angle)*j.y);
}

bool mayorpuntosY(cv::Point2f i, cv::Point2f j)
{
    cv::Point2f vertices[4];
    RectanguloR.points(vertices);
    i = i - vertices[indice];
    j = j - vertices[indice];

    return (cos(RectanguloR.angle)*i.y - sin(RectanguloR.angle)*i.x) < (cos(RectanguloR.angle)*j.y - sin(RectanguloR.angle)*j.x);
}

// Comparador para ordenar los puntos del patron int
bool cmp(std::pair<int,int> p1, std::pair<int,int> p2){
    if(p1.first!=p2.first)
        return p1.first<p2.first;
    return p1.second > p2.second;
}

// Comparador para hallar los puntos en una recta
bool cmp2(std::pair<float,cv::Point2f>  p1, std::pair<float,cv::Point2f> p2){
    return p1.first < p2.first;
}

PatternDetector::PatternDetector(QObject *parent) : QObject(parent)
{
    colors.push_back(MY_COLOR_RED);
    colors.push_back(MY_COLOR_BLUE);
    colors.push_back(MY_COLOR_YELLOW);
    colors.push_back(MY_COLOR_GREEN);
    colors.push_back(MY_COLOR_ORANGE);
    colors.push_back(MY_COLOR_WHITE);
}

void PatternDetector::setCurrentPattern(unsigned int pattType)
{
    currentPattern = pattType;
}

unsigned int PatternDetector::getCurrentPattern()
{
    return currentPattern;
}

void PatternDetector::setImage(cv::Mat im)
{
    img = im;
}

///
/// \brief PatternDetector::setVisualizer asigna el visualizador que se usara para el procesamiento
/// \param vis  MainWindow donde se visualizara los videos
///
void PatternDetector::setVisualizer(MainWindow *vis)
{
    visualizer = vis;
}

///
/// \brief PatternDetector::setSizePattern setea el tamaño del patrón
/// \param nRows    Numero de filas que tiene el patron
/// \param nCols    Numero de columnas que tiene el patron
///
void PatternDetector::setSizePattern(int nRows, int nCols)
{
    numRows = nRows;
    numCols = nCols;
    //trackGrid = new TrackingGrid(numRows * numCols);
}

///
/// \brief Funcion que aplica Thresholding Adaptativo, este algo está basado en el paper
/// "Adaptive Thresholding Using the Integral Image" de Derek Bradley y Gerhard Roth
/// \param input    Imagen en escala de grises que será segmentada
/// \return         Imagen binaria resultante
///
cv::Mat PatternDetector::adaptiveThresholdIntegralImage(cv::Mat input)
{
    // Ancho y altura de la imagen
    int w = input.cols;
    int h = input.rows;
    // Tamaño de la ventana S = (w/DIV_S)
    int s2 = (w / DIV_S) / 2;
    // Declaracion de variables auxiliares
    int sum = 0;
    int count = 0;
    int x1, x2, y1, y2;

    // Imagen integral
    unsigned long *intImg;
    intImg = (unsigned long *)malloc(sizeof(unsigned long)*h*w);

    // Imagen binaria de salida
    cv::Mat binImg(h, w, CV_8UC1);

    // Calculo de la imagen integral basado en los valores de los pixeles de input
    for (int i = 0; i < h; i++) {
        sum = 0;
        for(int j = 0; j < w; j++) {
            sum += input.at<uchar>(i,j);
            if (i == 0)
                intImg[i*w + j] = sum;
            else
                intImg[i*w + j] = intImg[(i-1)*w + j] + sum;
        }
    }

    // Se aplica thresholding y se obtiene la imagen binaria
    for (int i = 0; i < h; i++) {
        for(int j = 0; j < w; j++) {
            // Valores (x1,y1) y (x2,y2) de la ventana SxS
            x1 = j - s2;
            x2 = j + s2;
            y1 = i - s2;
            y2 = i + s2;

            // Verificación de bordes
            if(x1 < 0) x1 = 0;
            if(x2 >= w) x2 = w - 1;
            if(y1 < 0) y1 = 0;
            if(y2 >= h) y2 = h - 1;

            count = (x2 - x1) * (y2 - y1);
            sum = intImg[y2*w + x2] - intImg[y1*w + x2] - intImg[y2*w + x1] + intImg[y1*w + x1];

            // Proceso de binarización
            if((input.at<uchar>(i,j) * count) <= (sum * (1.0 - Tr)))
                binImg.at<uchar>(i,j) = 0;
            else
                binImg.at<uchar>(i,j) = 255;
        }
    }

    free(intImg);
    return binImg;
}

///
/// \brief PatternDetector::cleanNoiseCenters  se encarga de eliminar ruido haciendo analisis de los radios de los contornos hallados
/// \param vCenters Vector de centros de los contornos
/// \param vRadius  Vector de radios de los contornos
/// \return Vector de centros con una reduccion de ruido
///
std::vector<cv::Point2f> PatternDetector::cleanNoiseCenters(std::vector<cv::Point2f> vCenters, std::vector<std::pair<float, int> > vRadius, int maxError)
{
    // Si el numero de centros es el mismo numero de componentes del patron, se regresa el mismo vector
    if(vCenters.size() <= (numCols * numRows + maxError)) {
        // Ordenamiento de los radios en orden descendente para contar las frecuencias por intervalo
        if(vRadius.size()<=2)
            return vCenters;
        sort(vRadius.rbegin(), vRadius.rend());
        radioOptimo = (vRadius[0].first + vRadius[vRadius.size()-1].first) * 0.5;
        return vCenters;
    }

    std::vector<std::pair<int, float > > freqs;
    std::vector<std::pair<std::pair<float, float>, std::pair<int, int> > > extraInfo;
    float avgVal, stdVal;
    int modeVal, posMode;

    // Se obtiene las frecuencias de los datos agrupados
    switch (currentPattern) {
        case PATT_CIRCLE:
            metrics::getFrequences<float,int>(vRadius, freqs, extraInfo, false, 9);
            break;
        default:
            metrics::getFrequences<float,int>(vRadius, freqs, extraInfo, false);
            break;
    }
    // Se obtiene el promedio y la desviacion estandar
    metrics::getAvgStd(freqs, avgVal, stdVal);
    // Se obtiene la moda y el intervalo en que se encuentra
    metrics::getMode(freqs, modeVal, posMode);

    // Vector donde se almacenaran los centros del patron
    std::vector<cv::Point2f> keypoints;
    float minRad, maxRad;

    // Minimo y maximo radio que debe tener un circulo del patron
    if(modeVal == (int)(numCols * numRows)) {
        minRad = extraInfo[posMode].first.first - ERRORF;
        maxRad = extraInfo[posMode].first.second + ERRORF;
    }
    else {
        minRad = freqs[posMode].second - stdVal;
        maxRad = freqs[posMode].second + stdVal;
    }
    for(size_t i = 0; i < vRadius.size(); i++) {
        if(vRadius[i].first >= minRad && vRadius[i].first <= maxRad) {
            keypoints.push_back(vCenters[vRadius[i].second]);
        }
    }
    // El radio optimo es el label del intervalo moda
    radioOptimo = freqs[posMode].second;
    return keypoints;
}

///
/// \brief PatternDetector::findGrid    Encuentra los contornos de interes
/// \param image    Imagen binaria de entrada
/// \return Vector de centros de los contornos de interes
///
std::vector<cv::Point2f> PatternDetector::findGrid(cv::Mat image)
{
    // Obtencion de los contornos con jerarquia
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(image, contours, hierarchy ,CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    // Vector de centros
    std::vector<cv::Point2f> keypoints;

    // Variables auxiliares
    double areaPar, auxFactorPar, auxFactorCurr;
    int parent, child;
    cv::Point2f centerCurr, centerPar;

    std::vector<std::pair<float, int> > vectRadios;
    for(size_t i = 0; i < contours.size(); i++)
    {
        parent = hierarchy[i][3];
        child = hierarchy[i][2];

        if(child == -1) {
            if(parent != -1 && hierarchy[i][0] == -1 && hierarchy[i][1] == -1) {
                // PADRE: Rectangulo donde encaja la elipse o el contorno del padre
                cv::RotatedRect boxPar;
                if(contours[parent].size() < MIN_SIZE_CONTOUR)
                    boxPar = minAreaRect(contours[parent]);
                else {
                    cv::Mat pointsf;
                    cv::Mat(contours[parent]).convertTo(pointsf, CV_32F);
                    boxPar = fitEllipse(pointsf);
                }
                centerPar = boxPar.center;

                // ACTUAL: Rectangulo donde encaja la elipse o el contorno del actual
                cv::RotatedRect boxCurr;
                if(contours[i].size() < MIN_SIZE_CONTOUR)
                    centerCurr = centerPar;
                else {
                    cv::Mat pointsf;
                    cv::Mat(contours[i]).convertTo(pointsf, CV_32F);
                    boxCurr = fitEllipse(pointsf);
                    centerCurr = boxCurr.center;
                }

                // Calculo de areas
                areaPar = contourArea(contours[parent]);

                // Factor de aspect ratio
                auxFactorPar = std::min(boxPar.size.width, boxPar.size.height) / std::max(boxPar.size.width, boxPar.size.height);
                auxFactorCurr = std::min(boxCurr.size.width, boxCurr.size.height) / std::max(boxCurr.size.width, boxCurr.size.height);
                if(auxFactorPar < R_PAR_MIN_ASPECT_RATIO || auxFactorCurr < R_CHD_MIN_ASPECT_RATIO)
                    continue;

                // Factor de rectangularidad
                auxFactorPar = areaPar / boxPar.size.area();
                if (auxFactorPar < R_PAR_MIN_RECTAN)
                    continue;

                // Almacenamiento del centro de los anillos concentricos
                keypoints.push_back(cv::Point2f((centerCurr.x + centerPar.x) * 0.5, (centerCurr.y + centerPar.y) * 0.5));
                vectRadios.push_back(std::make_pair(std::max(boxPar.size.width, boxPar.size.height) * 0.5, keypoints.size() - 1));

                // Grafica de los contornos (padre e hijo)
                //drawContours(imgOut, contours, i, cv::Scalar::all(255), 1, 8);
                //drawContours(imgOut, contours, parent, cv::Scalar::all(255), 1, 8);
                // Grafica de las elipses
                //ellipse(imgOut, boxPar, cv::Scalar(0,0,255), 1, CV_AA);
                //ellipse(imgOut, boxCurr, cv::Scalar(0,0,255), 1, CV_AA);
            }
        }
    }
    keypoints = cleanNoiseCenters(keypoints, vectRadios, 2);
    // Grafica de los centros despues de la limpieza
    /*for(size_t i = 0; i < keypoints.size(); i++) {
        circle(imgOut, keypoints[i], 3, cv::Scalar(255,255,0), -1);
    }*/
    return keypoints;
}

bool PatternDetector::processingRingsPattern(std::vector<cv::Point2f> &keypoints, double &acc_t)
{
    // Variables auxiliares
    cv::Mat imgGray, imgBlur, imgThresh;
    double start_time, gray_time, blur_time, threshold_time, grid_time, conhull_time=0, pattern_time;
    char time[55];

    // Conversion de imagen a escala de grises
    start_time = omp_get_wtime();
    cv::cvtColor(img, imgGray, CV_BGR2GRAY);
    gray_time = omp_get_wtime() - start_time;

    // Aplicacion de filtro gaussiano
    start_time = omp_get_wtime();
    cv::GaussianBlur(imgGray, imgBlur, cv::Size(3,3), 0.5, 0.5);
    blur_time = omp_get_wtime() - start_time;

    // Segmentacion de imagen usando threshold adaptativo
    start_time = omp_get_wtime();
    imgThresh = adaptiveThresholdIntegralImage(imgBlur);
    threshold_time = omp_get_wtime() - start_time;

    // Obtención del ROI
    cv::Mat imgGrid = cv::Mat::zeros(img.size(), CV_8UC3);
    start_time = omp_get_wtime();
    keypoints = findGrid(imgThresh);
    grid_time = omp_get_wtime() - start_time;

    cv::Mat imgCH = img.clone();
    std::vector<cv::Point2f> corners;
    start_time = omp_get_wtime();
    convexHullCorners(keypoints, corners);
    conhull_time = omp_get_wtime() - start_time;

    start_time = omp_get_wtime();
    bool trackCorrect = trackingRingsPoints(keypoints, corners);
    pattern_time = omp_get_wtime() - start_time;

    std::sprintf(time, "Gray Time: %.3f", gray_time);
    putText(imgGray, time, cv::Point2f(15, 25), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255, 255), 2);
    visualizer->visualizeImage(PROC1, ImageHelper::convertMatToQimage(imgGray), "Color To Gray");

    std::sprintf(time, "Blur Time: %.3f", blur_time);
    putText(imgBlur, time, cv::Point2f(15, 25), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255, 255), 2);
    visualizer->visualizeImage(PROC2, ImageHelper::convertMatToQimage(imgBlur), "Filtro gaussiano");

    std::sprintf(time, "Threshold Time: %.3f", threshold_time);
    putText(imgThresh, time, cv::Point2f(15, 25), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255, 255), 2);
    visualizer->visualizeImage(PROC3, ImageHelper::convertMatToQimage(imgThresh), "Threshold adaptativo (paper)");

    std::sprintf(time, "Grid Time: %.3f", grid_time);
    putText(imgGrid, time, cv::Point2f(15, 25), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255, 255), 2);
    for(size_t i = 0; i < keypoints.size(); i++) {
        circle(imgGrid, keypoints[i], 3, cv::Scalar(255,255,0), -1);
    }
    visualizer->visualizeImage(PROC4, ImageHelper::convertMatToQimage(imgGrid), "Grid");

    std::sprintf(time, "Convex Hull Time: %.3f", conhull_time);
    putText(imgCH, time, cv::Point2f(15, 25), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255, 255), 2);
    for(int i = 0; i < corners.size(); i++){
        circle(imgCH, corners[i], 10, colors[i], CV_FILLED,8,0);
    }
    visualizer->visualizeImage(PROC5, ImageHelper::convertMatToQimage(imgCH), "Convex Hull");

    std::sprintf(time, "Pattern Time: %.3f - Total Time: %.3f", pattern_time, gray_time+blur_time+threshold_time+grid_time+conhull_time+pattern_time);
    putText(img, time, cv::Point2f(15, 25), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255, 255), 2);
    cv::drawChessboardCorners(img, cv::Size(numRows, numCols), keypoints, true);
    visualizer->visualizeImage(PROCFIN, ImageHelper::convertMatToQimage(img), "Pattern");

    acc_t += (gray_time+blur_time+threshold_time+grid_time+conhull_time+pattern_time);
    return trackCorrect;
}

bool PatternDetector::trackingRingsPoints(std::vector<cv::Point2f> &keypoints)
{
    if(keypoints.size() != numCols * numRows) {
        return false;
    }

    cv::RotatedRect rr = cv::minAreaRect(keypoints);
    RectanguloR = rr;

    //if (rr.angle < -90)
    //    cv::waitKey(19);
    if (rr.size.width < rr.size.height)
    {
        indice = 2;
        RectanguloR.angle = -(-90 - RectanguloR.angle)*PI / 180;
    }
    else
    {
        indice = 1;
        RectanguloR.angle = RectanguloR.angle*PI / 180;
        //RectanguloR.angle = 0;
    }

    std::sort(keypoints.begin(), keypoints.end(), mayorpuntosX);

    for (size_t i = 0; i < numCols; i++)
    {
        std::sort(keypoints.begin() + numRows * i, keypoints.begin() + (numRows * i + numRows), mayorpuntosY);
    }

    //std::sort(centros.begin(), centros.end(), mayorpuntosY);

    cv::circle(img, keypoints[0], 10, cv::Scalar(255, 0, 125));
    cv::putText(img,std::to_string(0),keypoints[0],cv::FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,255),2);

    for (size_t i = 1; i < numCols*numRows; i++)
    {
        cv::putText(img,std::to_string(i),keypoints[i],cv::FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,255),2);

        cv::circle(img, keypoints[i], 10, MY_COLOR_BLUE);

        cv::line(img, keypoints[i - 1], keypoints[i], MY_COLOR_GREEN);
    }

    visualizer->visualizeImage(PROCFIN, ImageHelper::convertMatToQimage(img), "Pattern");
    return true;
}

bool  PatternDetector::convexHullCorners(std::vector<cv::Point2f> &keypoints, std::vector<cv::Point2f> &corners)
{
    corners.clear();
    std::vector<std::vector<cv::Point2f> > hull(1);
    cv::convexHull(cv::Mat(keypoints), hull[0], false);

    //Obteniendo las esquinas del convexhull en el patron
    std::vector<int> posCornes = getPosCornes(hull);

    for(int i = 0; i < (int)posCornes.size(); i++){
        corners.push_back(hull[0][posCornes[i]]);
    }

    return true;
}

bool PatternDetector::trackingRingsPoints(std::vector<cv::Point2f> &keypoints, std::vector<cv::Point2f> &corners){

    if(keypoints.size() != numCols * numRows) {
        return false;
    }

    std::vector<std::pair<cv::Point2f,cv::Point2f> > extremosUpDown;
    // Hallando extremos, arriba y abajo
    for(int i = 0; i < corners.size(); i++){
        extremosUpDown.push_back(std::make_pair(corners[i],corners[(i+1) % 4]));
    }

    // Hallando una recta con 6 puntos en su contenido extremosUpDown
    std::vector<std::vector<std::pair<float,float> > > ans;
    for(int i=0;i<(int)extremosUpDown.size();i++){
        cv::Point2f A = extremosUpDown[i].first;
        cv::Point2f B = extremosUpDown[i].second;
        cv::Point2f P;
        // Interseccion de la recta AB con el punto P
        std::vector<std::pair<float,float> > aux;
        for(int k=0;k<(int)keypoints.size();k++){

            // Vemos que no sean los mismo puntos para evitar overflow
            if( (keypoints[k].x == A.x && keypoints[k].y == A.y ) || (keypoints[k].x == B.x && keypoints[k].y == B.y )) continue;
            P = keypoints[k];
            // Hallando la distancia del punto P a la recta AB
            double numerador = (P.x-A.x) * (B.y-A.y) - (P.y-A.y) * (B.x-A.x);
            double denominador = sqrt((B.x - A.x)*(B.x - A.x) + (B.y - A.y)*(B.y - A.y));
            double distancia = numerador / denominador;
            if(abs((int)distancia) < 6){ // se escoge 6 como tolerancia de precision
                aux.push_back(std::make_pair(keypoints[k].x,keypoints[k].y));
            }
        }
        aux.push_back(std::make_pair(A.x,A.y));
        aux.push_back(std::make_pair(B.x,B.y));

        if((int)aux.size()==numCols){
            //Ordenando Ascendentemente x, descendentemente y
            sort(aux.begin(),aux.end(),cmp);
            ans.push_back(aux);
        }
    }

    std::vector<std::pair<float,float> > SortPoints;
    std::stack<std::vector<std::pair<float,float> > > pila;
    // escribir lineas de colores
    if(ans.size()>1){

        cv::Point2f PPP = cv::Point2f(ans[0][0].first,ans[0][0].second);
        for(int j=0;j<std::min((int)ans[0].size(),(int)ans[1].size());j++){

            SortPoints.push_back(std::make_pair(ans[0][j].first,ans[0][j].second));
            SortPoints.push_back(std::make_pair(ans[1][j].first,ans[1][j].second));

            std::vector<std::pair<float,cv::Point2f> > distanciaRecta; // Distancia a la recta AB del punto P
            // Hallando los puntos de la recta AB
            cv::Point2f A =  cv::Point2f(ans[0][j].first,ans[0][j].second);
            cv::Point2f B =  cv::Point2f(ans[1][j].first,ans[1][j].second);
            cv::Point2f P;
            // Keypoints tiene todos los puntos del patron
            for(int k=0;k<(int)keypoints.size();k++){
                //Vemos que no sean los mismo puntos para evitar overflow
                if( (keypoints[k].x == A.x && keypoints[k].y == A.y ) || (keypoints[k].x == B.x && keypoints[k].y == B.y )) continue;
                P = keypoints[k];
                // Hallando la distancia del punto P a la recta AB
                double numerador = (P.x-A.x) * (B.y-A.y) - (P.y-A.y) * (B.x-A.x);
                double denominador = sqrt((B.x - A.x)*(B.x - A.x) + (B.y - A.y)*(B.y - A.y));
                double distancia = numerador / denominador;
                distanciaRecta.push_back(std::make_pair(abs((float)distancia),P));
            }

            // Ordenamos las distancias, para escoger los 3 mas cercanos
            std::sort(distanciaRecta.begin(),distanciaRecta.end(),cmp2);
            for(int i=0;i<numRows-2;i++){
                SortPoints.push_back(std::make_pair(distanciaRecta[i].second.x,distanciaRecta[i].second.y));
               // circle(img, distanciaRecta[i].second, 5, colors[j%colors.size()], CV_FILLED,8,0);
            }


            //circle(img, Point(ans[1][j].first,ans[1][j].second), 10, CV_RGB(0,0,0), CV_FILLED,8,0);
            std::sort(SortPoints.rbegin(), SortPoints.rend(), [](const std::pair<float, float>& first, const std::pair<float, float>& second){
                return (first.second < second.second);
            });
            // Almacenando los puntos de una recta
            pila.push(SortPoints);
            SortPoints.clear();
        }

        // Escribiendo las rectas de manera descendente
        //int counter = 0;  // Contador para etiquetar los puntos
        keypoints.clear();
        // Extraendo los elementos de la pila
        while(!pila.empty()){
            // Escribiendo numeros
            for(int i=0;i<pila.top().size();i++){
                //std::stringstream sstr;
                //sstr<<counter;
                //counter++;
                //cv::putText(img,sstr.str(),cv::Point2f(pila.top()[i].first,pila.top()[i].second),cv::FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,255),2);
                keypoints.push_back(cv::Point2f(pila.top()[i].first,pila.top()[i].second));
            }
            pila.pop();
        }
    }

    return true;
}

