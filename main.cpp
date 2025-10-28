#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "camera_parameters.h"

void printHelp() {
    std::cout << "Usage: ArUcoPoseEstimation.out [-f image_path] [-s] [-d]\n"
              << "Options:\n"
              << "  -h                Show this help message\n"
              << "  -f image_path     Path to the input image\n"
              << "  -s                Show image with detected markers and axes\n"
              << "  -d                Print detected marker ids and pose data\n";
}

int main(int argc, char** argv ) {
    Mat InputImage;
    bool SHOW_IMAGE = false;
    bool PRINT_DATA = false;

    /**************************************************************************************/
    /************************************Parse Input***************************************/
    /**************************************************************************************/

    int opt;
    while ((opt = getopt(argc, argv, "hf:sd")) != -1) {
        switch (opt) {
            case 'h':
                printHelp();
                return 0;
            case 'f':
                InputImage = imread(optarg, 1);
                if (!InputImage.data) {
                    std::cout << "No image data \n";
                    return -1;
                }
                break;
            case 's':
                SHOW_IMAGE = true;
                break;
            case 'd':
                PRINT_DATA = true;
                break;
            default:
                std::cerr << "Unknown option\n";
                printHelp();
                return -1;
        }
    }

    /**************************************************************************************/
    /************************************ArUco Detection***********************************/
    /**************************************************************************************/

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);

    // ArUco detection
    cv::aruco::detectMarkers(InputImage, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

    // Print out detected marker ids sorted
    if (PRINT_DATA) {
        std::sort(markerIds.begin(), markerIds.end());
        std::cout << "Detected marker ids: ";
        for (auto i : markerIds)
            std::cout << i << " ";
        std::cout << std::endl;
        std::cout << "Number of detected markers: " << markerIds.size() << std::endl;
    }

    /**************************************************************************************/
    /************************************Pose Estimation***********************************/
    /**************************************************************************************/

    float markerLength = 26.41f; // Marker side length in mm
    Mat cameraMatrix = OnePlus11CameraMatrix;
    Mat distCoeffs = OnePlus11DistCoeffs;
    std::vector<Vec3d> rvecs(markerIds.size()); // Rotation vectors
    std::vector<Vec3d> tvecs(markerIds.size()); // Translation vectors

    // set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

    // Estimate pose for each detected marker
    for (size_t i = 0; i < markerIds.size(); i++) {
        cv::solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs[i], tvecs[i]);
    }

    // Print out pose estimation results
    if (PRINT_DATA) {
        for (size_t i = 0; i < markerIds.size(); i++) {
            std::cout << "Rotation Vector: [" << rvecs[i][0] << ", " << rvecs[i][1] << ", " << rvecs[i][2] << "]" << std::endl;
            std::cout << "Translation Vector: [" << tvecs[i][0] << ", " << tvecs[i][1] << ", " << tvecs[i][2] << "]" << std::endl;
        }
    }

    // Display the image with detected markers and axes
    if (SHOW_IMAGE) {
        cv::aruco::drawDetectedMarkers(InputImage, markerCorners, markerIds);
        for (size_t i = 0; i < markerIds.size(); i++) {
            cv::drawFrameAxes(InputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
        }
        cv::namedWindow("Detected ArUco markers and pose", cv::WINDOW_NORMAL);
        cv::imshow("Detected ArUco markers and pose", InputImage);
        waitKey(0);
    }
    
    return 0;
}
