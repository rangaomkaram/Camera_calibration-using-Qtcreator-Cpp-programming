#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void createArucoMarkers();
    void processvideo();
    void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allfoundCorners, bool showResults);
    void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners);
    void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients);
    bool loadCameraCalibration(string name, Mat& cameraMatrix, Mat& distanceCoefficients);
    bool saveCameraCalibration(String name, Mat cameraMatrix, Mat distanceCoefficients);
    void CameraCalibrationProcess(Mat& cameraMatrix, Mat& distanceCoefficients);
    int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimension);


private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
