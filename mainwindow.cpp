#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace cv;
using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    int x,y;
    m_rectSize = 20;
    int dividend_factor = 10;
    m_FrameHeight = 0;
    m_FrameWidth = 0;
    cv::Rect prv_rect, next_rect;
    std::ofstream FileWrite;
    cv::namedWindow("previous image");
    cv::namedWindow("previous image with flow");
    cv::namedWindow("merged image");
    cv::Mat drawingPrv, drawingCurr;
    vector<Mat> merged_image;
    vector<Point2f> points[2];
    vector<uint> point_status;
    vector<Point2f> points_tracked;
    vector<pair <Rect, Rect> > rectangles;
    vector<pair <Rect, uint> > rectangle_status;
    vector<pair <Rect, Rect> > old_rectangles;
    int minHessian = 400;
    vector<Mat> points2d;
    Mat_<double> x1, x2;
    bool is_projective = true;
    cv::VideoWriter writer1, writer2;
    foldername   = "/home/terminalx/Projects/Endo_sfm_NCC_trial1/Results/";

    int npts;
    //xfeatures2d::SurfFeatureDetector surfdetector(minHessian);
    //xfeatures2d::SiftFeatureDetector siftdetector;
    //OrbFeatureDetector ORBdetector;
    Ptr<ORB> ORBdetector = ORB::create();
    //const string ORBdetector;
    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    Size subPixWinSize(10,10), winSize(31,31);
    m_videoHandle.open("/home/terminalx/crop_output_cut_2016-12-26_10-32-33,,,,2_l.avi");
    //m_videoHandle.open("/home/terminalx/Programs/opticalflow/Videos/LSD_room/LSD_room_short.avi");


    m_videoHandle >> m_currentFrame;
    cv::resize(m_currentFrame, m_currentFrame, Size(305,224));
    m_FrameHeight = m_currentFrame.rows;
    m_FrameWidth = m_currentFrame.cols;

    int count = 0;
    while(true)
    {
        vector<uchar> status;
        vector<float> err;
        size_t i, k;

        m_currentFrame.copyTo(m_previousFrame);
        m_videoHandle >> m_currentFrame;

        cv::resize(m_currentFrame, m_currentFrame, Size(305,224));
        if(m_currentFrame.empty())
            break;
        m_previousFrame.copyTo(drawingPrv);
        m_currentFrame.copyTo(drawingCurr);
        if(m_currentFrame.channels() == 3)
        {
            cvtColor(m_currentFrame, m_currentFrameGray, CV_BGR2GRAY);
        }
        if(m_previousFrame.channels() == 3)
        {
            cvtColor(m_previousFrame, m_previousFrameGray, CV_BGR2GRAY);
        }



        /// Create the result matrix
        int result_cols =  m_previousFrameGray.cols - m_rectSize + 1;
        int result_rows = m_previousFrameGray.rows - m_rectSize + 1;
        /// Localizing the best match with minMaxLoc
        double minVal; double maxVal; Point minLoc; Point maxLoc;
        Point matchLoc;
        Mat result;
        Point difference;
        float distance;
        RNG rng( 0xFFFFFFFF );
        double fps = m_videoHandle.get(CAP_PROP_FPS);
        cv::Size tamano((int)m_videoHandle.get(CAP_PROP_FRAME_WIDTH), (int)m_videoHandle.get(CAP_PROP_FRAME_HEIGHT));
        string output_filename1 = foldername + "/output_NCC.avi";
        writer1.open(output_filename1, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, tamano);
        result.create( result_rows, result_cols, CV_32FC1 );

        std::vector<KeyPoint> keypoints_object;

        Mat mask;

        // find the keypoints and descriptors with ORB
        //ORBdetector.detect(m_previousFrameGray, keypoints_object);
        if(count == 0)
        {
            points[0].clear();
            points[1].clear();
            for (y = 0; y < m_FrameHeight - m_rectSize; y = y + m_rectSize)
            {
                for (x = 0; x < m_FrameWidth - m_rectSize; x = x + m_rectSize)
                {
                    prv_rect = cv::Rect(x, y, m_rectSize, m_rectSize);
                    cv::Mat roi = m_previousFrameGray(prv_rect);
                    matchTemplate( m_currentFrameGray, roi, result, CV_TM_CCORR_NORMED );
                    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

                    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
                    matchLoc = maxLoc;


                    difference = Point2f(prv_rect.x,prv_rect.y) - Point2f(matchLoc.x, matchLoc.y);

                    distance = sqrt( difference.ddot(difference));
                    //qDebug() << "Distance: " << distance;

                    if(distance > 20.0)
                    {
                        next_rect = cv::Rect(-1, -1, -1 , -1);
                        rectangle_status.push_back(make_pair(next_rect, 0));
                        points[0].push_back(Point2f(-1,-1));
                        points[1].push_back(Point2f(-1, -1));
                        point_status.push_back(0);
//                        for(int k = 0; k < m_rectSize/dividend_factor; k++)
//                        {
//                            for(int j = 0; j < m_rectSize/dividend_factor; j++)
//                            {
//                                points[0].push_back(Point2f(-1, -1));
//                                points[1].push_back(Point2f(-1, -1));
//                                point_status.push_back(0);
//                            }
//                        }
                    }
                    else
                    {
                        next_rect = cv::Rect(matchLoc.x, matchLoc.y, m_rectSize ,  m_rectSize);
                        points[0].push_back(Point2f((prv_rect.x + (m_rectSize/2)), (prv_rect.y + (m_rectSize/2))));
                        points[1].push_back(Point2f((matchLoc.x + (m_rectSize/2)), (matchLoc.y + (m_rectSize/2))));
                        rectangle_status.push_back(make_pair(next_rect, 1));
                        point_status.push_back(1);

//                        for(int k = 0; k < m_rectSize/dividend_factor; k++)
//                        {
//                            for(int j = 0; j < m_rectSize/dividend_factor; j++)
//                            {


////                                int x_random = rng.uniform(prv_rect.x , prv_rect.x + m_rectSize);
////                                int y_random = rng.uniform(prv_rect.y , prv_rect.y + m_rectSize);
////                                int x_diff = x_random - prv_rect.x;
////                                int y_diff = y_random - prv_rect.y;
////                                points[0].push_back(Point(x_random, y_random));
////                                points[1].push_back(Point(matchLoc.x + x_diff, matchLoc.y + y_diff));

////                                prvPoints.push_back(Point2f((x + k), (y + j)));
////                                currPoints.push_back(Point2f((matchLoc.x + k), (matchLoc.y + j)));
//                                points[0].push_back(Point2f((x + k), (y + j)));
//                                points[1].push_back(Point2f((matchLoc.x + k), (matchLoc.y + j)));
//                                point_status.push_back(1);
//                            }
//                        }
                    }
                    rectangles.push_back(make_pair(prv_rect, next_rect));
                }
            }
            old_rectangles = rectangles;
        }

        //        if(count == 0)
        //        {
        //            ORBdetector->detect(m_previousFrameGray, keypoints_object, mask);

        //            for(int i = 0; i < keypoints_object .size(); i++)
        //            {
        //                points[0].push_back( keypoints_object [i].pt );
        //            }
        //            cout<< "Number of good features in the frame ->" << points[0].size() << endl;
        //        }

        //        calcOpticalFlowPyrLK(m_previousFrameGray, m_currentFrameGray, points[0], points[1], status, err, winSize,
        //                3, termcrit, 0, 0.001);
        //        cout << "Number of features tracked in the frame -> " << points[1].size() << endl;


        else
        {
            int prev_index0 = 0;
            int prev_index1 = 0;
            bool prev_index_in0 = false;
            bool prev_index_in1 = true;
            int index_rectangle = 0;

            for(uint i = 0 ; i < rectangle_status.size(); i++)
            {
                index_rectangle = i;
                //int index_rectangle = floor(i/(m_rectSize * m_rectSize));
               // qDebug()<< "Index rectangle   " << index_rectangle;
                if(point_status[index_rectangle] == 0)
                {
                    points[1].push_back(Point2f(-1, -1));

//                    if(prev_index0 != index_rectangle)
//                    {
//                        prev_index_in0 = true;
//                    }
//                    if(prev_index_in0 == true)
//                    {
//                        prev_index_in0 = false;
//                        prev_index0 = index_rectangle;

//                        for(int k = 0; k < m_rectSize/dividend_factor; k++)
//                        {
//                            for(int j = 0; j < m_rectSize/dividend_factor; j++)
//                            {
//                                points[1].push_back(Point2f(-1, -1));
//                            }
//                        }
                   // }
                }
                else
                {
//                    if(prev_index1 != index_rectangle)
//                    {
//                        prev_index_in1 = true;
//                    }
//                    if(prev_index_in1 == true)
//                    {
//                        prev_index_in1 = false;
//                        prev_index1 = index_rectangle;
                        prv_rect = cv::Rect((rectangle_status[index_rectangle].first.x), (rectangle_status[index_rectangle].first.y), m_rectSize, m_rectSize);
                        cv::Mat roi = m_previousFrameGray(prv_rect);
                        matchTemplate( m_currentFrameGray, roi, result, CV_TM_CCORR_NORMED );
                        normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

                        minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
                        matchLoc = maxLoc;


                        difference = Point2f(prv_rect.x,prv_rect.y) - Point2f(matchLoc.x, matchLoc.y);

                        distance = sqrt( difference.ddot(difference));
                        //qDebug() << "Distance: " << distance;

                        if(distance > 20.0)
                        {
                            next_rect = cv::Rect(-1, -1, -1 , -1);
                            rectangle_status[index_rectangle].first = next_rect;
                            rectangle_status[index_rectangle].second = 0;

                            points[1].push_back(Point2f(-1, -1));
                            point_status[index_rectangle] = 0;
//                            for(int k = 0; k < m_rectSize/dividend_factor; k++)
//                            {
//                                for(int j = 0; j < m_rectSize/dividend_factor; j++)
//                                {
//                                    points[1].push_back(Point2f(-1, -1));
//                                    point_status[index_rectangle+k+j] = 0;
//                                   // qDebug() << "Point_status   " << point_status[index_rectangle+k+j];
//                                }
//                            }
                        }
                        else
                        {
                            next_rect = cv::Rect(matchLoc.x, matchLoc.y, m_rectSize ,  m_rectSize);
                            points[1].push_back(Point2f((matchLoc.x + (m_rectSize/2)), (matchLoc.y + (m_rectSize/2))));
                            point_status[index_rectangle] = 1;
//                            for(int k = 0; k < m_rectSize/dividend_factor; k++)
//                            {
//                                for(int j = 0; j < m_rectSize/dividend_factor; j++)
//                                {
//                                    int x_random = rng.uniform(prv_rect.x , prv_rect.x + m_rectSize);
//                                    int y_random = rng.uniform(prv_rect.y , prv_rect.y + m_rectSize);
//                                    int x_diff = x_random - prv_rect.x;
//                                    int y_diff = y_random - prv_rect.y;
//                                    //points[0].push_back(Point(x_random, y_random));
//                                    points[1].push_back(Point(matchLoc.x + x_diff, matchLoc.y + y_diff));
//                                    //points[1].push_back(Point2f((matchLoc.x + k), (matchLoc.y + j)));
//                                    point_status[index_rectangle+k+j] = 1;
//                                    //qDebug() << "Point_status ->  " << point_status[index_rectangle+k+j];
//                                }
//                            }
                            rectangle_status[index_rectangle].first = next_rect;
                            rectangle_status[index_rectangle].second = 1;
                        }
                        rectangles.push_back(make_pair(prv_rect, next_rect));
                  //  }

                }

            }
            old_rectangles.clear();
            //point_status.clear();
            old_rectangles = rectangles;
        }



        merged_image.push_back(m_previousFrame);

        npts = points[1].size();
        x1 = Mat_<double>(2, npts);
        x2 = Mat_<double>(2, npts);

        for(int i = 0; i < npts; i++)
        {

            if(point_status[i] == 1)
            {
                x1(0, i) = static_cast<double>(points[0][i].x);
                x1(1, i) = static_cast<double>(points[0][i].y);

                //                x2(0, i) = static_cast<double>(points[1][i].x);
                //                x2(1, i) = static_cast<double>(points[1][i].y);
            }
            else
            {
                x1(0, i) = static_cast<double>(-1);
                x1(1, i) = static_cast<double>(-1);
            }
        }






        points2d.push_back(Mat(x1));
        //points2d.push_back(Mat(x2));

        //cout << "Number of features seen consisitently -> " << points2d.size() << endl;

        for( i = k = 0; i < points[1].size(); i++ )
        {

            if(points[1][i].x == -1 || points[0][i].x == -1)
            {

            }
            else
            {
                circle( drawingPrv, points[1][i], 2, Scalar(0,0,0), -1, 8);
                line(drawingPrv, points[0][i],points[1][i],Scalar(255,0,0));
            }
        }


        cv::imshow("previous image", m_previousFrame);
        cv::imshow("previous image with flow", drawingPrv);
 //       cv::imshow("current image", drawingCurr);


//        std::ostringstream name;
//        name << foldername << "im_" << count << ".png";
//        imwrite(name.str(), drawingPrv);

        //writer1 << drawingPrv;



        if(count == 5)
        {

            Matx33d K_in = Matx33d( 450, 0, 458,
                                    0, 450, 337,
                                    0, 0,  1);
//            Matx33d K_in = Matx33d( 500, 0, 320,
//                                    0, 500, 240,
//                                    0, 0,  1);

            //cv::imshow("merged image", merged_image);
            vector<Mat> Rs_est, ts_est, points3d_estimated;
            sfm::reconstruct(points2d, Rs_est, ts_est, K_in, points3d_estimated, is_projective);
            //sfm::reconstruct(points2d, Rs_est, ts_est, K_in, points3d_estimated, is_projective);
            // Print output

            cout << "\n----------------------------\n" << endl;
            cout << "Reconstruction: " << endl;
            cout << "============================" << endl;
            cout << "Estimated 3D points: " << points3d_estimated.size() << endl;
            cout << "Estimated cameras: " << Rs_est.size() << endl;
            cout << "Refined intrinsics: " << endl << K_in << endl << endl;
            cout << "3D Visualization: " << endl;
            cout << "============================" << endl;




            /// Create 3D windows

            viz::Viz3d window("Coordinate Frame");
            window.setWindowSize(Size(1000,1000));
            window.setWindowPosition(Point(300,300));
            window.setBackgroundColor(); // black by default

            // Create the pointcloud
            cout << "Recovering points  ... ";

            // recover estimated points3d
            vector<Vec3f> point_cloud_est;
            for (int i = 0; i < points3d_estimated.size(); ++i)
                point_cloud_est.push_back(Vec3f(points3d_estimated[i]));


            //FileWrite.open("/home/terminalx/Pointcloud_lsdroom.obj");
//            for(int i = 0; i < point_cloud_est.size(); i++)
//            {
//                FileWrite << "v " << point_cloud_est[i][0] << " " <<  point_cloud_est[i][1] << " " << point_cloud_est[i][3] << endl;
//            }
            cout << "[DONE]" << endl;


            /// Recovering cameras
            cout << "Recovering cameras ... ";

            vector<Affine3d> path;
            for (size_t i = 0; i < Rs_est.size(); ++i)
                path.push_back(Affine3d(Rs_est[i],ts_est[i]));

            cout << "[DONE]" << endl;


            /// Add the pointcloud
            if ( point_cloud_est.size() > 0 )
            {
                cout << "Rendering points   ... ";

                viz::WCloud cloud_widget(point_cloud_est, viz::Color::green());
                window.showWidget("point_cloud", cloud_widget);

                cout << "[DONE]" << endl;
            }
            else
            {
                cout << "Cannot render points: Empty pointcloud" << endl;
            }


            /// Add cameras
            if ( path.size() > 0 )
            {
                cout << "Rendering Cameras  ... ";

                window.showWidget("cameras_frames_and_lines", viz::WTrajectory(path, viz::WTrajectory::BOTH, 0.1, viz::Color::green()));
                window.showWidget("cameras_frustums", viz::WTrajectoryFrustums(path, K_in, 0.1, viz::Color::yellow()));

                window.setViewerPose(path[1]);

                cout << "[DONE]" << endl;
            }
            else
            {
                cout << "Cannot render the cameras: Empty path" << endl;
            }

            /// Wait for key 'q' to close the window
            cout << endl << "Press 'q' to close each windows ... " << endl;

            window.spin();
            count = -1;
            points2d.clear();
            rectangle_status.clear();
            point_status.clear();
            //points[0].clear();
        }




        char ch = waitKey(2);



        if(ch == 'q')
        {
            continue;
        }
        std::swap(points[0], points[1]);
        points[1].clear();
        rectangles.clear();

        count++;
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}
