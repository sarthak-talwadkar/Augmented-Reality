/*
    Sarthak Uday Talwadkar
    Date : March 11, 2025
    Detection of corners using Harris Corner detection algorithm
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;
using namespace cv;
/*
    Projecting on 2 patterns at the same time
*/
vector<Point3f> generateCylinder(Point3f baseCenter, float radius, float height, int segments)
{
    vector<Point3f> points;
    for (int i = 0; i < segments; i++)
    {
        float angle = 2 * CV_PI * i / segments;
        float x = baseCenter.x + radius * cos(angle);
        float y = baseCenter.y + radius * sin(angle);
        points.push_back(Point3f(x, y, baseCenter.z));
        points.push_back(Point3f(x, y, baseCenter.z + height));
    }
    return points;
}

vector<Point3f> generatePyramid(Point3f baseCenter, float baseWidth, float height)
{
    float halfWidth = baseWidth / 2;
    return {
        {baseCenter.x - halfWidth, baseCenter.y - halfWidth, baseCenter.z},
        {baseCenter.x + halfWidth, baseCenter.y - halfWidth, baseCenter.z},
        {baseCenter.x + halfWidth, baseCenter.y + halfWidth, baseCenter.z},
        {baseCenter.x - halfWidth, baseCenter.y + halfWidth, baseCenter.z},
        {baseCenter.x, baseCenter.y, baseCenter.z + height}};
}

int main(int argc, char *argv[])
{
    // Define multiple chessboard patterns
    vector<Size> patternSizes = {Size(9, 6), Size(7, 5)};
    vector<vector<Vec3f>> pointSets;

    // Generate 3D points for each pattern
    for (const auto &patternSize : patternSizes)
    {
        vector<Vec3f> pointSet;
        for (int i = 0; i < patternSize.height; i++)
        {
            for (int j = 0; j < patternSize.width; j++)
            {
                pointSet.push_back(Vec3f(i, j, 0));
            }
        }
        pointSets.push_back(pointSet);
    }

    // Load camera calibration parameters
    FileStorage fs("Calibration.yml", FileStorage::READ);
    if (!fs.isOpened())
    {
        cerr << "Error opening file !!" << endl;
        return -1;
    }

    Mat cameraMatrix, distortionCoefficient;
    fs["cameraMatix"] >> cameraMatrix;
    fs["distortionCoeficients"] >> distortionCoefficient;
    fs.release();

    VideoCapture cap(2);
    if (!cap.isOpened())
        return -2;

    Mat frame, gray;
    while (true)
    {
        cap >> frame;
        if (frame.empty())
            break;

        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Process each pattern
        for (size_t p = 0; p < patternSizes.size(); p++)
        {
            vector<Point2f> corners;
            bool foundCorner = findChessboardCorners(gray, patternSizes[p], corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

            if (foundCorner)
            {
                cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.001));

                Mat rvec, tvec;
                solvePnP(pointSets[p], corners, cameraMatrix, distortionCoefficient, rvec, tvec);

                // Project an image or 3D object onto this pattern
                vector<Point3f> virtualObject;
                if (p == 0)
                {
                    // Project a cylinder for the first pattern
                    virtualObject = generateCylinder(Point3f(3, 4, 2), 0.5f, 3.0f, 20);
                }
                else if (p == 1)
                {
                    // roject a pyramid for the second pattern
                    virtualObject = generatePyramid(Point3f(3, 4, 2), 1.0f, 2.0f);
                }

                vector<Point2f> projectedPoints;
                projectPoints(virtualObject, rvec, tvec, cameraMatrix, distortionCoefficient, projectedPoints);

                // Draw the projected object
                if (p == 0)
                {
                    // Draw cylinder
                    for (size_t i = 0; i < projectedPoints.size(); i += 2)
                    {
                        line(frame, projectedPoints[i], projectedPoints[i + 1], Scalar(0, 0, 255), 2);
                    }
                }
                else if (p == 1)
                {
                    // Draw pyramid
                    vector<int> pyramidConnections = {0, 1, 1, 2, 2, 3, 3, 0, 0, 4, 1, 4, 2, 4, 3, 4};
                    for (size_t i = 0; i < pyramidConnections.size(); i += 2)
                    {
                        line(frame, projectedPoints[pyramidConnections[i]], projectedPoints[pyramidConnections[i + 1]], Scalar(0, 255, 0), 2);
                    }
                }
            }
        }

        namedWindow("Multiple Pattern Detection", 0);
        imshow("Multiple Pattern Detection", frame);
        if (waitKey(10) == 27)
            break;
    }
    return 0;
}