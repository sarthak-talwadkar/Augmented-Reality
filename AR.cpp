/*
    Sarthak Uday Talwadkar
    Data : March 8th, 2025
    Project a virtual object using checkerboard pattern
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;
using namespace cv;

/*
    Generates a cylinder coordinates
    Inputs are Center coordinates for the each cylinder,  its radius, height and number of segments
    Output is the points for each cylinder
*/
vector<Point3f> generateCylinder(Point3f baseCenter, float radius, float height, int segments)
{
    vector<Point3f> points;
    for (int i = 0; i < segments; i++)
    {
        float angle = 2 * CV_PI * i / segments;
        float x = baseCenter.x + radius * cos(angle);
        float y = baseCenter.y + radius * sin(angle);
        points.push_back(Point3f(x, y, baseCenter.z));          // Base circle
        points.push_back(Point3f(x, y, baseCenter.z + height)); // Top circle
    }
    return points;
}

int main(int argc, char *argv[])
{
    // Defining the pattern size for the checkerboard
    int cols = 9;
    int rows = 6;
    Size patternSize(cols, rows);

    // Defining the World coordinates
    vector<Vec3f> pointSet;
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            pointSet.push_back(Vec3f(i, j, 0));
        }
    }

    // Reading the values from calibration file and saving the values in variables which will be used later for error correction
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

    // Defining the Virtual objects start
    // Cylinders and a cuboid
    float pillarRadius = 0.5f;
    float pillarHeight = 3.0f;
    float cubeSize = 3.0f;
    int cylinderSegment = 50;

    vector<Point3f> axisPoints = {
        {0, 0, 0}, {3, 1, 1}, {1, 3, 1}, {1, 1, 3}, {2.5, 3.5, 1}};

    vector<Point3f> virtualObject;
    vector<Point3f> pillarCenter = {
        {4, 6, 0},
        {4, 3, 0},
        {1, 6, 0},
        {1, 3, 0},
        {3.5f, 4.5f, pillarHeight + 2}};
    for (auto &center : pillarCenter)
    {
        auto cylinderPoint = generateCylinder(center, pillarRadius, pillarHeight, cylinderSegment);
        // virtualObject.insert(virtualObject.end(), cylinderPoint.begin(), cylinderPoint.end());
    }

    float minX = 6, maxX = 0;
    float minY = 2, maxY = 7;
    float cubeZend = pillarHeight + 2, cubeZstart = pillarHeight;
    vector<Point3f> cubePoints = {
        // Bottom face (covers pillar bases)
        {minX, minY, cubeZend},
        {maxX, minY, cubeZend},
        {maxX, maxY, cubeZend},
        {minX, maxY, cubeZend},

        // Top face (covers pillar tops)
        {minX, minY, cubeZstart},
        {maxX, minY, cubeZstart},
        {maxX, maxY, cubeZstart},
        {minX, maxY, cubeZstart}};
    // virtualObject.insert(virtualObject.end(), cubePoints.begin(), cubePoints.end());

    vector<Point3f> pyramidPoints = {
        {minX - 1, minY + 1, (cubeZend + pillarHeight)},
        {maxX + 1, minY + 1, (cubeZend + pillarHeight)},
        {maxX + 1, maxY - 1, (cubeZend + pillarHeight)},
        {minX - 1, maxY - 1, (cubeZend + pillarHeight)},

        // Apex point
        {{3.5f, 4.5f, (cubeZend + pillarHeight + 2)}}};
    // virtualObject.insert(virtualObject.end(), pyramidPoints.begin(), pyramidPoints.end());

    // Add Batman logo points to the virtual object
    virtualObject.insert(virtualObject.end(), batmanLogoPoints.begin(), batmanLogoPoints.end());
    // Defining the Virtual objects end

    // Reading the video stream
    VideoCapture cap(2);
    if (!cap.isOpened())
        return -2;

    Mat frame, gray;
    vector<Point2f> corners;

    cout << "Camera Matrix : " << cameraMatrix << endl;
    cout << "Distortion Coefficient" << distortionCoefficient << endl;

    while (true)
    {
        cap >> frame;
        if (frame.empty())
            break;

        cvtColor(frame, gray, COLOR_BGR2GRAY);
        bool foundCorner = findChessboardCorners(gray, patternSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

        if (foundCorner)
        {
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.001));

            if (!corners.empty())
            {
                ostringstream oss;
                oss << "First Corner: (" << corners[0].x << ", " << corners[0].y << ")";
                // putText(frame, oss.str(), Point(20, 60),
                // FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
                circle(frame, corners[0], 5, Scalar(0, 255, 255), -1);
            }

            Mat rvec, tvec;
            solvePnP(pointSet, corners, cameraMatrix, distortionCoefficient, rvec, tvec);

            // Projecting a point outside of corners
            vector<Point2f> projectedAxis;
            projectPoints(axisPoints, rvec, tvec, cameraMatrix, distortionCoefficient, projectedAxis);

            // circle(frame, projectedAxis[4], 5, Scalar(100, 0, 255), -1);
            // stringstream pose;
            // pose << "(x,y,z) : " << "(" << projectedAxis[4].x << ", " << projectedAxis[4].y << ")";
            // putText(frame, pose.str(), projectedAxis[4], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1);

            // Draw axes lines
            line(frame, projectedAxis[0], projectedAxis[1], Scalar(0, 0, 255), 2); // X - Red
            line(frame, projectedAxis[0], projectedAxis[2], Scalar(0, 255, 0), 2); // Y - Green
            line(frame, projectedAxis[0], projectedAxis[3], Scalar(255, 0, 0), 2); // Z - Blue

            // Axis labels
            putText(frame, "X", projectedAxis[1], FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
            putText(frame, "Y", projectedAxis[2], FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
            putText(frame, "Z", projectedAxis[3], FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 0), 2);

            // Projecting a virtual objects
            vector<Point2f> projectedPoints;
            projectPoints(virtualObject, rvec, tvec, cameraMatrix, distortionCoefficient, projectedPoints);

            // Draws cylinders
            int pointsPerCylinder = cylinderSegment * 2;
            for (int c = 0; c < 5; c++)
            {
                int startIdx = c * pointsPerCylinder;
                for (int i = 0; i < cylinderSegment; i++)
                {
                    // Vertical lines
                    line(frame, projectedPoints[startIdx + i * 2],
                         projectedPoints[startIdx + i * 2 + 1], Scalar(0, 255, 0), 1);

                    // Horizontal circles
                    int next_i = (i + 1) % cylinderSegment;
                    line(frame, projectedPoints[startIdx + i * 2],
                         projectedPoints[startIdx + next_i * 2], Scalar(0, 255, 0), 1); // Base
                    line(frame, projectedPoints[startIdx + i * 2 + 1],
                         projectedPoints[startIdx + next_i * 2 + 1], Scalar(0, 255, 0), 1); // Top
                }
            }

            // Draw cube (last 8 points)
            vector<int> cubeConnections = {0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7};
            for (size_t i = 0; i < cubeConnections.size(); i += 2)
            {
                line(frame,
                     projectedPoints[5 * pointsPerCylinder + cubeConnections[i]],
                     projectedPoints[5 * pointsPerCylinder + cubeConnections[i + 1]],
                     Scalar(0, 255, 255), 2);
            }

            // Draw a Pyramid
            vector<int> pyramidConnections = {0, 1, 1, 2, 2, 3, 3, 0, 0, 4, 1, 4, 2, 4, 3, 4};
            for (size_t i = 0; i < pyramidConnections.size(); i += 2)
            {
                line(frame,
                     projectedPoints[5 * pointsPerCylinder + 8 + pyramidConnections[i]],
                     projectedPoints[5 * pointsPerCylinder + 8 + pyramidConnections[i + 1]],
                     Scalar(255, 255, 0), 2);
            }

            stringstream poseText, poseText1;
            poseText << "Rot: " << format(rvec, Formatter::FMT_DEFAULT);
            putText(frame, poseText.str(), Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
            poseText1 << "Trans: " << format(tvec, Formatter::FMT_DEFAULT);
            putText(frame, poseText1.str(), Point(20, 60),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
        }

        namedWindow("Camera Pose Estimation", 0);
        imshow("Camera Pose Estimation", frame);
        if (waitKey(10) == 27)
            break;
    }
    return 0;
}