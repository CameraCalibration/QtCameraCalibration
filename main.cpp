#include <QApplication>
#include "mainwindow.h"
#include "settings.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    //! [file_read]
    Settings s;
    const std::string inputSettingsFile = argc > 1 ? argv[1] : "D:/opt/windows/Microsoft/VisualStudio/repos/CameraCalibration/QtCameraCalibration/settings.xml";
    cv::FileStorage fs(inputSettingsFile, cv::FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        std::cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << std::endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file
    //! [file_read]

    //FileStorage fout("settings.yml", FileStorage::WRITE); // write config as YAML
    //fout << "Settings" << s;

    if (!s.goodInput)
    {
        std::cout << "Invalid input detected. Application stopping. " << std::endl;
        return -1;
    }

    MainWindow w(s);
    w.resize(1305, 840);
    w.setMinimumSize(1305, 840);
    w.move(20, 20);
    w.show();

    return a.exec();
}
