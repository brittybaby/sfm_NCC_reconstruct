#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>




#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>
//#include <opencv2/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio/videoio.hpp"
//#include "opencv2/core/mat.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include "QDebug"
#include <QImage>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;



    int m_rectSize;
    int m_FrameHeight;
    int m_FrameWidth;
    std::string foldername;
    cv::VideoCapture m_videoHandle;
    cv::Mat m_currentFrame, m_previousFrame, m_currentFrameGray, m_previousFrameGray, img_roi;
};

#endif // MAINWINDOW_H
