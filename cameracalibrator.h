#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include <QObject>
#include <string>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "settings.h"

class MainWindow;
class PatternDetector;

class CameraCalibrator : public QObject
{
    Q_OBJECT
public:
    explicit CameraCalibrator(Settings s, QObject *parent = nullptr);
    void setVisualizer(MainWindow *vis);
    void setActived(bool act);
    void setCurrentCalibrator(unsigned int calib);
    void setSizePattern(int nRows, int nCols);
    bool loadVideo(std::string path);
    void initProcessing(unsigned int pattSelected);
    bool runCalibrationAndSave(Settings& s, cv::Size imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                               std::vector<std::vector<cv::Point2f> > imagePoints);
    double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> > &objectPoints,
                                     const std::vector<std::vector<cv::Point2f> > &imagePoints,
                                     const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
                                     const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                     std::vector<float> &perViewErrors, bool fisheye);
    void calcBoardCornerPositions(cv::Size boardSize, float squareSize, std::vector<cv::Point3f> &corners,
                                  Settings::Pattern patternType);
    bool runCalibration( Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                         std::vector<std::vector<cv::Point2f> > imagePoints, std::vector<cv::Mat>& rvecs,
                         std::vector<cv::Mat>& tvecs, std::vector<float>& reprojErrs,  double& totalAvgErr);
    void saveCameraParams( Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                           const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                           const std::vector<float>& reprojErrs, const std::vector<std::vector<cv::Point2f> >& imagePoints,
                           double totalAvgErr );


private:
    void processingPattern();
    void processingRingsGrid();

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
    Settings config;

    // Additional parameters to calibration
    bool isCalibrated;      // Estado de la calibracion
    unsigned int currCalib; // Calibrador actual (Ninguno, Opencv o Ankur)
    unsigned int currFrameSelector;     // Selector de frames actual (Manual, por intervalos, ransac)
    bool saveCamParams;     // Flag para indicar si se guarda o no los parametros y otras salidas del proceso de calibracion
};

#endif // CAMERACALIBRATOR_H
