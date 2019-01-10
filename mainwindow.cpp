#include "mainwindow.h"
#include "cameracalibrator.h"

MainWindow::MainWindow(Settings s, QWidget *parent) : QMainWindow(parent)
{
    phbxLayout1 = new QBoxLayout(QBoxLayout::LeftToRight);
        numCols = new QLineEdit();
        numRows = new QLineEdit();
        phbxLayout1->addWidget(numCols);
        phbxLayout1->addWidget(numRows);

    phbxLayout2 = new QBoxLayout(QBoxLayout::LeftToRight);
        qvbl1 = new QBoxLayout(QBoxLayout::TopToBottom);
            title1 = new QLabel("Fase 1");
            img1 = new QLabel();
            title1->setStyleSheet("font-size:8pt; font-weight:600; color:#000000;");
            title1->setAlignment(Qt::AlignCenter);
            img1->setStyleSheet("border: 1px solid gray");
            //img1->setMaximumSize(QSize(261,231));
            img1->setMinimumSize(QSize(261,231));
            qvbl1->addWidget(title1);
            qvbl1->addWidget(img1);
        qvbl2 = new QBoxLayout(QBoxLayout::TopToBottom);
            title2 = new QLabel("Fase 2");
            img2 = new QLabel();
            title2->setStyleSheet("font-size:8pt; font-weight:600; color:#000000;");
            title2->setAlignment(Qt::AlignCenter);
            img2->setStyleSheet("border: 1px solid gray");
            //img2->setMaximumSize(QSize(261,231));
            img2->setMinimumSize(QSize(261,231));
            qvbl2->addWidget(title2);
            qvbl2->addWidget(img2);
        qvbl3 = new QBoxLayout(QBoxLayout::TopToBottom);
            title3 = new QLabel("Fase 3");
            img3 = new QLabel();
            title3->setStyleSheet("font-size:8pt; font-weight:600; color:#000000;");
            title3->setAlignment(Qt::AlignCenter);
            img3->setStyleSheet("border: 1px solid gray");
            //img3->setMaximumSize(QSize(261,231));
            img3->setMinimumSize(QSize(261,231));
            qvbl3->addWidget(title3);
            qvbl3->addWidget(img3);
        qvbl4 = new QBoxLayout(QBoxLayout::TopToBottom);
            title4 = new QLabel("Fase 4");
            img4 = new QLabel();
            title4->setStyleSheet("font-size:8pt; font-weight:600; color:#000000;");
            title4->setAlignment(Qt::AlignCenter);
            img4->setStyleSheet("border: 1px solid gray");
            //img4->setMaximumSize(QSize(261,231));
            img4->setMinimumSize(QSize(261,231));
            qvbl4->addWidget(title4);
            qvbl4->addWidget(img4);
        qvbl5 = new QBoxLayout(QBoxLayout::TopToBottom);
            title5 = new QLabel("Fase 5");
            img5 = new QLabel();
            title5->setStyleSheet("font-size:8pt; font-weight:600; color:#000000;");
            title5->setAlignment(Qt::AlignCenter);
            img5->setStyleSheet("border: 1px solid gray");
            //img5->setMaximumSize(QSize(261,231));
            img5->setMinimumSize(QSize(261,231));
            qvbl5->addWidget(title5);
            qvbl5->addWidget(img5);

        phbxLayout2->addItem(qvbl1);
        phbxLayout2->addItem(qvbl2);
        phbxLayout2->addItem(qvbl3);
        phbxLayout2->addItem(qvbl4);
        phbxLayout2->addItem(qvbl5);

    phbxLayout0 = new  QBoxLayout(QBoxLayout::LeftToRight);
        qvbl0 = new QBoxLayout(QBoxLayout::TopToBottom);
            title0 = new QLabel("Results");
            img0 = new QLabel();
            title0->setStyleSheet("font-size:10pt; font-weight:600; color:#000000;");
            title0->setAlignment(Qt::AlignCenter);
            img0->setStyleSheet("border: 1px solid gray");
            //img0->setMaximumSize(QSize(522,462));
            img0->setAlignment(Qt::AlignCenter);
            img0->setMinimumSize(QSize(522,462));
            qvbl0->addWidget(title0);
            qvbl0->addWidget(img0);
        phbxLayout0->addLayout(qvbl0);

    pbxLayout = new QBoxLayout(QBoxLayout::TopToBottom);
        pbxLayout->addLayout(phbxLayout1);
        pbxLayout->addLayout(phbxLayout2);
        pbxLayout->addLayout(phbxLayout0);

    // Set layout in QWidget
    window = new QWidget();
    window->setLayout(pbxLayout);

    // Set QWidget as the central layout of the main window
    setCentralWidget(window);

    createActions();
    createMenus();

    QString message = tr("A context menu is available by right-clicking");
    statusBar()->showMessage(message);

    calibrator = new CameraCalibrator(s);
    calibrator->setVisualizer(this);
    on_rbRing_clicked();
}

void MainWindow::createActions()
{
    loadAct = new QAction(tr("&Load"), this);
    loadAct->setShortcuts(QKeySequence::New);
    loadAct->setStatusTip(tr("Load a Video"));
    loadAct->setChecked(true);
    connect(loadAct, &QAction::triggered, this, &MainWindow::loadVideo);

    startAct = new QAction(tr("&Start"), this);
    startAct->setShortcuts(QKeySequence::New);
    startAct->setStatusTip(tr("Start the Process"));
    startAct->setEnabled(false);
    connect(startAct, &QAction::triggered, this, &MainWindow::startProcess);

    stopAct = new QAction(tr("&Stop"), this);
    stopAct->setShortcuts(QKeySequence::New);
    stopAct->setStatusTip(tr("Stop the Process"));
    stopAct->setEnabled(false);
    connect(stopAct, &QAction::triggered, this, &MainWindow::stopProcess);

    exitAct = new QAction(tr("&Exit"), this);
    exitAct->setShortcuts(QKeySequence::New);
    exitAct->setStatusTip(tr("Exit the Program"));
    startAct->setEnabled(false);
    connect(exitAct, &QAction::triggered, this, &QWidget::close);
}

void MainWindow::createMenus()
{
    fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(loadAct);
    fileMenu->addAction(startAct);
    fileMenu->addAction(stopAct);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);
}

#ifndef QT_NO_CONTEXTMENU
void MainWindow::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu menu(this);
    menu.addAction(loadAct);
    menu.addAction(startAct);
    menu.exec(event->globalPos());
}
#endif // QT_NO_CONTEXTMENU

void MainWindow::loadVideo()
{
    QString pathImage = QFileDialog::getOpenFileName(this, tr("Search video"), "", tr("Video Files (*.avi *.mp4 *.wmv)"));

    if(!calibrator->loadVideo(pathImage.toStdString())) {
        //ui->lblMsg->setText("Video is not found");
        startAct->setEnabled(false);
    }
    else {
        loadAct->setStatusTip(tr(""));
        startAct->setEnabled(true);
    }
}

void MainWindow::startProcess()
{
    startAct->setEnabled(false);
    stopAct->setEnabled(true);

    calibrator->setSizePattern(numRows->text().toInt(), numCols->text().toInt());
    calibrator->setCurrentCalibrator(currCalibrator);
    calibrator->initProcessing(pattSelected);
}

void MainWindow::stopProcess()
{
    stopAct->setEnabled(false);
    startAct->setEnabled(true);

    calibrator->setActived(false);
}

void MainWindow::visualizeImage(int id, QImage img, std::string title)
{
    QPixmap image = QPixmap::fromImage(img);
    switch (id) {
    case PROCFIN:
        img0->setPixmap(image.scaled(img0->size(), Qt::KeepAspectRatio));
        if(title == "") title = "RESULTADO FINAL";
        title0->setText(QString::fromStdString(title));
        break;
    case PROC1:
        img1->setPixmap(image.scaled(img1->size()));
        title1->setText(QString::fromStdString(title));
        break;
    case PROC2:
        img2->setPixmap(image.scaled(img2->size()));
        title2->setText(QString::fromStdString(title));
        break;
    case PROC3:
        img3->setPixmap(image.scaled(img3->size()));
        title3->setText(QString::fromStdString(title));
        break;
    case PROC4:
        img4->setPixmap(image.scaled(img4->size()));
        title4->setText(QString::fromStdString(title));
        break;
    case PROC5:
        img5->setPixmap(image.scaled(img5->size()));
        title5->setText(QString::fromStdString(title));
        break;
    }
}

void MainWindow::on_rbRing_clicked()
{
    pattSelected = PATT_RING;
    numRows->setText("4");
    numCols->setText("5");
}

void MainWindow::cleanImage(int id)
{
    switch (id) {
    case PROCFIN:
        img0->clear();
        break;
    case PROC1:
        img1->clear();
        title1->setText("");
        break;
    case PROC2:
        img2->clear();
        title2->setText("");
        break;
    case PROC3:
        img3->clear();
        title3->setText("");
        break;
    case PROC4:
        img4->clear();
        title4->setText("");
        break;
    case PROC5:
        img5->clear();
        title5->setText("");
        break;
    }
}
