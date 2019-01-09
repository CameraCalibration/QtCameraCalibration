#ifndef CONSTANTS_H
#define CONSTANTS_H

// Parametros para visualizacion
#define PROCFIN 0
#define PROC1   1
#define PROC2   2
#define PROC3   3
#define PROC4   4
#define PROC5   5
#define PROC6   6
#define PROC7   7
#define PROC8   8
#define PROC9   9

// Parametros de seleccion de patron
#define PATT_CIRCLE 10
#define PATT_RING   11

#define PI 3.14159265

// Parametros del thresholding
#define DIV_S   12
#define Tr       0.15f

// Parametros de contornos
#define MIN_SIZE_CONTOUR    6
#define MYEPS 1e-8

// Operaciones
#define dbg(x) cout<<#x<<"="<<x<<endl
#define dbg2(x,y) cout<<#x<<"="<<x<<" "<<#y<<"="<<y<<endl
#define MY_COLOR_YELLOW CV_RGB(255,255,0)
#define MY_COLOR_RED CV_RGB(255,0,0)
#define MY_COLOR_WHITE CV_RGB(255,255,255)
#define MY_COLOR_ORANGE CV_RGB(255,69,0)
#define MY_COLOR_ORANGE1 CV_RGB(100,100,0)
#define MY_COLOR_GREEN CV_RGB(20,150,20)
#define MY_COLOR_BLUE CV_RGB(0,0,205)
#define INF (1<<30)
#define MAXN 100

// Constantes usadas para los circulos
#define C_THRES_CANNY       100     // Threshold usado en Canny
#define C_FACTOR_CANNY      3       // Factor de Canny

#define C_MIN_ASPECT_RATIO  0.5     // Relacion 2:1 entre el largo y ancho del bounding box
#define C_MIN_RECTAN        0.7     // Rectangularidad (circulo -> 0,7853975)
#define C_MIN_AREA          0.2


// Constantes usadas para los anillos
#define R_PAR_MIN_ASPECT_RATIO      0.5     // Factor aspect ratio minimo del anillo padre
#define R_CHD_MIN_ASPECT_RATIO      0.4
#define R_CUR_MIN_ASPECT_RATIO      0.55
#define R_PAR_MIN_RECTAN    0.7     // Factor de rectangularidad
#define R_CHD_MIN_RECTAN    0.4
#define R_CUR_MIN_RECTAN    0.75
#define R_CUR_MIN_AREA      0.1

// Tipos de calibrador
#define CALIB_NONE      20
#define CALIB_OPENCV    21
#define CALIB_ANKUR     22

// Tipos de selector de frames
#define FRAMESEL_MANUAL     30
#define FRAMESEL_INTERVAL   31
#define FRAMESEL_RANSAC     32

#endif // CONSTANTS_H
