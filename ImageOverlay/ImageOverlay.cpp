/*
    Sarthak Uday Talwadkar
    Date : March 11, 2025
    Overlaying a Image onto the pattern
*/
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

/*
    Find the corner using findChessboardCorners and refining the corners using cornerSubPix
    Then adjust the image with warpPerspective to confine the image to detected corners
    and then overlay the region onto the frame
*/
int main()
{
    const int cols = 9;
    const int rows = 6;
    Size patternSize(cols, rows);

    // Load overlay image
    Mat overlayImage = imread("/home/sunny/Downloads/h1.jpg", IMREAD_UNCHANGED);
    if (overlayImage.empty())
    {
        cerr << "Error loading overlay image!" << endl;
        return -1;
    }

    VideoCapture cap(3);
    if (!cap.isOpened())
    {
        cerr << "Error opening camera!" << endl;
        return -1;
    }

    // Load calibration data
    FileStorage fs("Calibration.yml", FileStorage::READ);
    Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    Mat frame, gray;
    vector<Point2f> corners;

    while (true)
    {
        cap >> frame;
        if (frame.empty())
            break;

        // Detect checkerboard
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        bool found = findChessboardCorners(gray, patternSize, corners,
                                           CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

        if (found)
        {
            // Refine corners
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.001));

            // Get four outer corners
            vector<Point2f> boardCorners = {
                corners[0], corners[cols - 1],
                corners[cols * rows - 1], corners[cols * (rows - 1)]};

            // Define source points
            vector<Point2f> imgCorners = {
                Point2f(0, 0),
                Point2f(overlayImage.cols - 1, 0),
                Point2f(overlayImage.cols - 1, overlayImage.rows - 1),
                Point2f(0, overlayImage.rows - 1)};

            // Calculate homography matrix
            Mat H = findHomography(imgCorners, boardCorners);

            // Warp overlay image
            Mat warpedImage;
            warpPerspective(overlayImage, warpedImage, H, frame.size());
            imshow("Image Overlay5", warpedImage);

            // Create mask from channel
            Mat mask;
            cvtColor(warpedImage, mask, COLOR_BGR2GRAY);
            threshold(mask, mask, 1, 255, THRESH_BINARY);

            // Apply overlay
            Mat roi;
            bitwise_and(frame, frame, roi, ~mask);
            imshow("Image Overlay3", ~mask);
            add(roi, warpedImage, frame);
        }

        imshow("Image Overlay", frame);
        if (waitKey(10) == 27)
            break;
    }

    return 0;
}