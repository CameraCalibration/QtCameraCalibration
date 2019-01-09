#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include <QObject>
#include <string>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>

class MainWindow;
class PatternDetector;

class CameraCalibrator : public QObject
{
    Q_OBJECT
public:
    explicit CameraCalibrator(QObject *parent = nullptr);
    void setVisualizer(MainWindow *vis);
    void setActived(bool act);
    void setCurrentCalibrator(unsigned int calib);
    void setSizePattern(int nRows, int nCols);
    bool loadVideo(std::string path);
    void initProcessing(unsigned int pattSelected);

private:
    void processingPattern();
    void processingRingsGrid();

public:
signals:

public slots:

private:
    PatternDetector *pattDetector;
    std::string pathVideo;
    std::string folderOutVideo;
    unsigned int numCols;
    unsigned int numRows;
    cv::VideoCapture video;
    MainWindow *visualizer;
    bool actived;

    // Additional parameters to calibration
    bool isCalibrated;      // Estado de la calibracion
    unsigned int currCalib; // Calibrador actual (Ninguno, Opencv o Ankur)
    unsigned int currFrameSelector;     // Selector de frames actual (Manual, por intervalos, ransac)
    bool saveCamParams;     // Flag para indicar si se guarda o no los parametros y otras salidas del proceso de calibracion
};

#endif // CAMERACALIBRATOR_H
