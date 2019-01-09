#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QContextMenuEvent>
#include <QStatusBar>
#include <QImage>
#include <QWidget>
#include <QLabel>
#include <QBoxLayout>
#include <QPixmap>
#include <iostream>
#include <QColor>
#include <QPushButton>
#include <QLineEdit>
#include <QCheckBox>
#include <QComboBox>
#include <QProgressBar>
#include <QDebug>
#include <QGroupBox>
#include <QButtonGroup>
#include <QFileDialog>
#include <opencv2/opencv.hpp>
#include "constants.h"

class CameraCalibrator;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);

    void visualizeMsg(std::string msg);
    void visualizeValue(std::string label, double value);
    void visualizeImage(int id, QImage img, std::string title="");
    void cleanImage(int id);

private:
    void createActions();
    void createMenus();

public:
signals:

public slots:

private slots:
    void loadVideo();
    void startProcess();
    void stopProcess();
    void on_rbRing_clicked();

protected:
#ifndef QT_NO_CONTEXTMENU
    void contextMenuEvent(QContextMenuEvent *event) override;
#endif // QT_NO_CONTEXTMENU

private:
    QMenu *fileMenu;
    QActionGroup *alignmentGroup;
    QAction *loadAct;
    QAction *startAct;
    QAction *stopAct;
    QAction *exitAct;

    QAction *leftAlignAct;
    QAction *rightAlignAct;
    QAction *justifyAct;
    QAction *centerAct;

    QBoxLayout* phbxLayout1;
    QLineEdit *numRows, *numCols;
    QBoxLayout* phbxLayout2;
    QBoxLayout *qvbl1;
    QLabel *title1;
    QLabel *img1;
    QBoxLayout *qvbl2;
    QLabel *title2;
    QLabel *img2;
    QBoxLayout *qvbl3;
    QLabel *title3;
    QLabel *img3;
    QBoxLayout *qvbl4;
    QLabel *title4;
    QLabel *img4;
    QBoxLayout *qvbl5;
    QLabel *title5;
    QLabel *img5;

    QBoxLayout *phbxLayout0;
    QBoxLayout *qvbl0;
    QLabel *title0;
    QLabel *img0;

    QBoxLayout *pbxLayout;
    QWidget *window;

    CameraCalibrator *calibrator;
    unsigned int pattSelected;
    unsigned int currCalibrator;
    unsigned int currFrameSelector;

};

#endif // MAINWINDOW_H
