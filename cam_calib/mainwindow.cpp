#include "mainwindow.h"
#include "ui_mainwindow.h"
//opencv
#include <opencv2/video.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include "opencv2/core.hpp"
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <sstream>
#include <fstream>

using namespace cv;
using namespace std;

// Global variables
Mat frame1;
int keyboard; //input from keyboard
const float calibrationSquareDimension=0.0255f; //size of chessboard fields in [m]
const float arucoSquareDimension=0.05f; //size of our future aruco markers
const Size chessboardDimensions=Size(9,6); //this is the crosses between the chessboard fields
Mat cameraMatrix = Mat::eye(3,3,CV_64F);  //creates a Matlab-style identity matrix
Mat distanceCoefficients;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    processvideo();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::processvideo()
{
    //createArucoMarkers();
    CameraCalibrationProcess(cameraMatrix, distanceCoefficients);
    //loadCameraCalibration("Cameracalibration", cameraMatrix, distanceCoefficients);
    //startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimension);
}

void MainWindow::createArucoMarkers()
{
    Mat outputMarker;

    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    for(int i = 0; i < 50; i++)
    {
        aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
        ostringstream convert;
        string imageName="4x4Marker_";
        convert << imageName << i << ".jpg";
        imwrite(convert.str(), outputMarker);
    }
}


void MainWindow::createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners)
{
    for(int i =0; i < boardSize.height; i++)
    {
        for (int j=0; j< boardSize.width; j++)
        {
            corners.push_back(Point3f(j*squareEdgeLength, i*squareEdgeLength, 0.0f));
        }
    }
}


void MainWindow::getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allfoundCorners, bool showResults)
{
    for (vector<Mat>::iterator iter=images.begin(); iter !=images.end(); iter++)
    {
        vector<Point2f> pointBuf;
        bool found = findChessboardCorners(*iter, Size(9,6), pointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

        if (found)
        {
            allfoundCorners.push_back(pointBuf);
        }
        if(showResults)
        {
            drawChessboardCorners(*iter, Size(9,6), pointBuf, found);
            imshow("Looking for Corners",*iter);
            waitKey();
        }
    }
}

void MainWindow::cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients)
{
    vector<vector<Point2f>>checkerboardImageSpacePoints;
    getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false);

    vector<vector<Point3f>> worldSpaceCornerPoints(1);

    createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
    worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

    vector<Mat> rVectors, tVectors;
    distanceCoefficients=Mat::zeros(8,1,CV_64F);

    calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);
}


bool MainWindow::saveCameraCalibration(String name, Mat cameraMatrix, Mat distanceCoefficients)
{
    ofstream outStream(name);
    if(outStream)
    {
        uint16_t rows=cameraMatrix.rows;
        uint16_t columns=cameraMatrix.cols;

        outStream<<rows<<endl;
        outStream<<columns<<endl;


        for(int r = 0; r < rows; r++)
        {
            for(int c = 0; c < columns; c++)
            {
                double value = cameraMatrix.at<double>(r,c);
                outStream<<value<<endl;
            }
        }
        rows = distanceCoefficients.rows;
        columns = distanceCoefficients.cols;

        outStream<<rows<<endl;
        outStream<<columns<<endl;

        for(int r = 0; r < rows; r++)
        {
            for(int c = 0; c < columns; c++)
            {
                double value = distanceCoefficients.at<double>(r,c);
                outStream<<value<<endl;
            }
        }
        outStream.close();
        return true;
    }
    return false;
}


bool MainWindow::loadCameraCalibration(string name, Mat& cameraMatrix, Mat& distanceCoefficients)
{
    ifstream inStream(name);
    if (inStream)
    {
        uint16_t rows;
        uint16_t columns;
        inStream>>rows;
        inStream>>columns;

        cameraMatrix=Mat(Size(columns, rows), CV_64F);

        for(int r=0; r<rows; r++)
        {
            for(int c=0; c < columns; c++)
            {
                double read=0.0f;
                inStream>>read;
                cameraMatrix.at<double>(r,c)=read;
                cout<<cameraMatrix.at<double>(r,c)<<"\n";
            }
        }
        //Distance Coefficients
        inStream>>rows;
        inStream>>columns;

        distanceCoefficients=Mat::zeros(rows, columns, CV_64F);

        for(int r=0; r<rows; r++)
        {
            for(int c=0; c < columns; c++)
            {
                double read=0.0f;
                inStream>>read;
                distanceCoefficients.at<double>(r,c)=read;
                cout<<distanceCoefficients.at<double>(r,c)<< "\n";
            }
        }
        inStream.close();
        return true;
    }
    return false;
}

void MainWindow::CameraCalibrationProcess(Mat& cameraMatrix, Mat& distanceCoefficients)
{

    Mat frame;
    Mat drawToFrame;
    vector<Mat> savedImages;
    namedWindow("Webcam", WINDOW_AUTOSIZE);

    VideoCapture vid(0);

    if(!vid.isOpened())
    {
        cout<<"ERROR ACQUIRING VIDEO FEED\n";
        exit(EXIT_FAILURE);
    }

    while(true)
    {
        if(!vid.read(frame))
            break;

        vector<Vec2f> foundPoints;
        bool found = false;

        found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

        frame.copyTo(drawToFrame);
        drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);
        if(found)
            imshow("Webcam", drawToFrame);
        else
            imshow("Webcam", frame);


        switch(waitKey(10))
        {
        case' ':
            //saving image by pressing SPACE
            if(found)
            {
                Mat temp;
                frame.copyTo(temp);
                savedImages.push_back(temp);
                cout<<"Foto aufgenommen. Anzahl:"<<savedImages.size()<<endl;
            }
            break;
        case 13:
            //start calibration process of saved images by pressing ENTER
            if(savedImages.size()>15)
            {
                cout<<"Kalibrierung gestartet."<<endl;
                cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
                saveCameraCalibration("Cameracalibration", cameraMatrix, distanceCoefficients);
                cout<<"Kalibrierung gespeichert."<<endl;
            }
            break;
        case 27:
            //exit by pressing ESC
            destroyAllWindows();
            cout<<"Programm beendet."<<endl;
            return;
        }

    }
}
