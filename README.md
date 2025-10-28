# Program Overview
This program detects ArUco markers in a given image and estimates their pose using OpenCV. It supports optional display of the detected markers, printing of marker IDs and pose data, and can measure and log the execution time for detection and pose estimation. Timing results can be printed to the console or saved to a CSV file for further analysis.

It also features an utility program `calibrate_camera.cpp` that calculates the camera matrix and distortion coefficients given a set of chessboard images (OpenCV Calibration Pattern).

# Prerequisites
* Assummes the host has installed OpenCV 
    * Installing libopencv-dev using apt should be enough
    * Alternatively one can download and build OpenCV straight from the source (See https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html for a guide on how to do that).
* CMake
* gcc

# How to build and run the project
1. cd <project_folder>
2. cmake .
3. make
4. output/<name_of_executable> <list_of_args>

# Useful resouces
* Starting point for OpenCV project with gcc and CMake: https://docs.opencv.org/2.4/doc/tutorials/introduction/linux_gcc_cmake/linux_gcc_cmake.html#linux-gcc-usage
* AruCo detection and pose estimation example code: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
* OpenCV camera calibration: https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html
* Installing OpenCV on Linux: https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html 
