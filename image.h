#ifndef IMAGE_H
#define IMAGE_H

#include <opencv2/opencv.hpp>
#include <QImage>
#include <iostream>

namespace ImageHelper {
///
/// \brief convertMatToQimage convierte la imagen en una imagen de clase QImage.
/// \param input    Imagen que se convertira en QImage
/// \return         Imagen de clase QImage
///
inline QImage convertMatToQimage(cv::Mat input)
{
    // 8-bits unsigned, NO. OF CHANNELS=1
    if(input.type()==CV_8UC1)
    {
        // Set the color table (used to translate colour indexes to qRgb values)
        QVector<QRgb> colorTable;
        for (int i=0; i<256; i++)
            colorTable.push_back(qRgb(i,i,i));
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)input.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, input.cols, input.rows, input.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        return img;
    }
    // 8-bits unsigned, NO. OF CHANNELS=3
    if(input.type()==CV_8UC3)
    {
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)input.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, input.cols, input.rows, input.step, QImage::Format_RGB888);
        return img.rgbSwapped();
    }
    else
    {
        std::cout << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }
}

///
/// \brief convertQImageToMat   convierte una imagen QImage a imagen de tipo Mat de OpenCV
/// \param inImage              Imagen QImage que será transformada
/// \param inCloneImageData     Bool que indica si el contenido de la imagen será clonado
/// \return     Imagen de clase Mat
///
inline cv::Mat convertQImageToMat( const QImage &inImage, bool inCloneImageData = true )
{
    cv::Mat out;
    switch ( inImage.format() )
    {
     // 8-bit, 4 channel
     case QImage::Format_RGB32:
     {
        cv::Mat  mat( inImage.height(), inImage.width(), CV_8UC4, const_cast<uchar*>(inImage.bits()), inImage.bytesPerLine() );

        //return (inCloneImageData ? mat.clone() : mat);
        out = (inCloneImageData ? mat.clone() : mat);
        return out;
     }

     // 8-bit, 3 channel
     case QImage::Format_RGB888:
     {
        if ( !inCloneImageData )
           std::cout << "ASM::QImageToCvMat() - Conversion requires cloning since we use a temporary QImage";

        QImage   swapped = inImage.rgbSwapped();

        //return cv::Mat( swapped.height(), swapped.width(), CV_8UC3, const_cast<uchar*>(swapped.bits()), swapped.bytesPerLine() ).clone();
        out = cv::Mat( swapped.height(), swapped.width(), CV_8UC3, const_cast<uchar*>(swapped.bits()), swapped.bytesPerLine() ).clone();
        return out;
     }

     // 8-bit, 1 channel
     case QImage::Format_Indexed8:
     {
        cv::Mat  mat( inImage.height(), inImage.width(), CV_8UC1, const_cast<uchar*>(inImage.bits()), inImage.bytesPerLine() );

        out = (inCloneImageData ? mat.clone() : mat);
        return out;
     }

     default:
        std::cout << "ASM::QImageToCvMat() - QImage format not handled in switch:" << inImage.format();
        break;
    }
    return cv::Mat();
}

}

#endif // IMAGE_H
