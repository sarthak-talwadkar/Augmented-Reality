# Augmented-Reality

Overview

This repository contains four C++ projects leveraging OpenCV for computer vision and augmented reality (AR) applications. The implementations include Harris Corner Detection, 3D object projection using chessboard patterns, multi-pattern AR, and image overlay via homography. These projects demonstrate fundamental computer vision techniques, camera calibration, and real-time AR rendering.

Features
    HarrisCorner.cpp
        Real-time Harris Corner Detection on video feed.
        Computes gradients, applies Gaussian blur, and highlights corners using adaptive thresholding.
        Visualizes corners with red circles on the frame.

    AR.cpp
        Projects 3D virtual objects (cylinders, cube, pyramid) onto a detected chessboard pattern.
        Estimates camera pose using solvePnP and renders objects with projectPoints.
        Includes coordinate axes visualization and pose text overlay.

    multipleAR.cpp
        Detects two chessboard patterns (9x6 and 7x5) simultaneously.
        Projects a cylinder on the first pattern and a pyramid on the second.
        Uses separate 3D point sets and dynamic object rendering.

    ImageOverlay.cpp
        Overlays an image onto a chessboard using perspective warping.
        Computes homography with findHomography and blends the image using masking.

Technologies Used
    OpenCV 4.x (core, imgproc, calib3d modules)
    C++
    Camera calibration (Calibration.yml required).

References
    OpenCV Documentation
    Harris Corner : 
      https://docs.opencv.org/3.4/d4/d7d/tutorial_harris_detector.html
      https://stackoverflow.com/questions/50355747/how-to-extract-keypoints-from-harris-corner-detector-using-opencv
    Homography :
      https://learnopencv.com/homography-examples-using-opencv-python-c/
      https://stackoverflow.com/questions/18723751/opencv-displaying-an-image-over-a-video-frame-with-chessboard-rotation-of-th
    "Learning OpenCV" by Adrian Kaehler and Gary Bradski
