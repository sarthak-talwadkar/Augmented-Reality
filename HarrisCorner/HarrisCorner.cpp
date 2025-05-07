/*
    Sarthak Uday Talwadkar
    Date : March 11, 2025
    Detection of corners using Harris Corner detection algorithm
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

/*
    The program captures video from a camera, applies the Harris Corner Detection algorithm,
    and marks detected corners on the video frames.
*/
int main(int argc, char *argv[])
{
    VideoCapture cap(2);
    if (!cap.isOpened())
    {
        cerr << "Camera not found" << endl;
        return -1;
    }

    while (true)
    {
        Mat frame;
        cap >> frame;
        if (frame.empty())
        {
            cerr << "Cannot read the Frame" << endl;
            break;
        }

        int blockSize = 2;
        int apertureSize = 3;
        double k1 = 0.04;
        Mat gray, dst, dst_norm, dst_norm_scaled;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        gray.convertTo(gray, CV_32F);

        cornerHarris(gray, dst, blockSize, apertureSize, k1);
        normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
        convertScaleAbs(dst_norm, dst_norm_scaled);

        imshow("Harris Corner 1", dst_norm_scaled);

        // Calculation of gradient in x and y
        Mat Ix,
            Iy;
        Sobel(gray, Ix, CV_32F, 1, 0, 3);
        Sobel(gray, Iy, CV_32F, 0, 1, 3);

        Mat IxSq = Ix.mul(Ix);
        Mat IySq = Iy.mul(Iy);
        Mat Ixy = Ix.mul(Iy);

        // Applying blur to reduce noise
        int windowSize = 5;
        double sigma = 2.0;
        GaussianBlur(IxSq, IxSq, Size(windowSize, windowSize), sigma);
        GaussianBlur(IySq, IySq, Size(windowSize, windowSize), sigma);
        GaussianBlur(Ixy, Ixy, Size(windowSize, windowSize), sigma);

        // Harris Corner Algorithm
        Mat det = IxSq.mul(IySq) - Ixy.mul(Ixy);
        Mat trace = IxSq + IxSq;
        float k = 0.04f;
        Mat harrisResponse;
        max(det - k * trace.mul(trace), 0, harrisResponse);

        Mat dilated;
        dilate(harrisResponse, dilated, Mat());
        Mat localMax = (harrisResponse == dilated);
        Mat suppressed;
        harrisResponse.copyTo(suppressed);
        suppressed.setTo(0, localMax == 0);

        double threshold = 0.01 * suppressed.at<float>(0, 0);
        minMaxLoc(suppressed, 0, &threshold);
        threshold *= 0.01;
        Mat corners;
        compare(suppressed, threshold, corners, CMP_GT);

        // Showing the detected corner points on the current frame
        vector<Point> points;
        findNonZero(corners, points);
        for (size_t i = 0; i < points.size(); i++)
        {
            circle(frame, points[i], 3, Scalar(0, 0, 255), -1);
        }

        imshow("Harris Corner", frame);

        // Press Esc to exit
        if (waitKey(10) == 27)
        {
            break;
        }
    }

    return 0;
}