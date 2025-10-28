#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <chrono>
#include <iostream>
#include <fstream>

#include "camera_parameters.h"

void printHelp() {
    std::cout << "Usage: ArUcoPoseEstimation.out [options]\n"
              << "Options:\n"
              << "  -h                          Show this help message\n"
              << "  -f image_path               Path to the input image\n"
              << "  -s                          Show image with detected markers and axes\n"
              << "  -d                          Print detected marker ids and pose data\n"
              << "  -t                          Print detection and pose estimation time\n"
              << "  -c csv_path                 Save timing information to CSV file at csv_path\n";
}

int main(int argc, char** argv ) {
    // Program options
    Mat InputImage;
    bool SHOW_IMAGE = false;
    bool PRINT_DATA = false;
    bool OUTPUT_TIME = false;
    std::string CSV_PATH;

    // Time points for performance measurement
    // Steady clock is monotonic and not affected by system clock changes
    // There exists high_resolution_clock, but its steadiness is not guaranteed
    std::chrono::steady_clock::time_point pre_detection_time, post_detection_time;
    std::chrono::steady_clock::time_point pre_pose_time, post_pose_time;

    /**************************************************************************************/
    /************************************Parse Input***************************************/
    /**************************************************************************************/

    int opt;
    while ((opt = getopt(argc, argv, "hf:sdtc:")) != -1) {
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
            case 't':
                OUTPUT_TIME = true;
                break;
            case 'c':
                CSV_PATH = std::string(optarg);
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

    // Start detection timer
    pre_detection_time = std::chrono::steady_clock::now();

    // Aruco detection variables
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);

    // ArUco detection
    cv::aruco::detectMarkers(InputImage, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

    // End detection timer
    post_detection_time = std::chrono::steady_clock::now();

    /**************************************************************************************/
    /************************************Pose Estimation***********************************/
    /**************************************************************************************/

    // Start pose estimation timer
    pre_pose_time = std::chrono::steady_clock::now();

    // Pose estimation variables
    float markerLength = 26.41f; // Marker side length in mm
    Mat cameraMatrix = OnePlus11CameraMatrix;
    Mat distCoeffs = OnePlus11DistCoeffs;
    std::vector<Vec3d> rvecs(markerIds.size()); // Rotation vectors
    std::vector<Vec3d> tvecs(markerIds.size()); // Translation vectors

    // Set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength/2.f, -markerLength/2.f, 0);
    
    // Estimate pose for each detected marker
    for (size_t i = 0; i < markerIds.size(); i++) {
        cv::solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs[i], tvecs[i]);
    }

    // End pose estimation timer
    post_pose_time = std::chrono::steady_clock::now();

    /**************************************************************************************/
    /************************************Output Results************************************/
    /**************************************************************************************/

    // Print out program data
    if (PRINT_DATA) {
        // Print out detected marker ids
        std::sort(markerIds.begin(), markerIds.end());
        std::cout << "Detected marker ids: ";
        for (auto i : markerIds)
            std::cout << i << " ";
        std::cout << std::endl;
        std::cout << "Number of detected markers: " << markerIds.size() << std::endl;

        // Print out pose estimation results
        for (size_t i = 0; i < markerIds.size(); i++) {
            std::cout << "Rotation Vector: [" << rvecs[i][0] << ", " << rvecs[i][1] << ", " << rvecs[i][2] << "]" << std::endl;
            std::cout << "Translation Vector: [" << tvecs[i][0] << ", " << tvecs[i][1] << ", " << tvecs[i][2] << "]" << std::endl;
        }
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

    // Calculate timing durations
    auto detection_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(post_detection_time - pre_detection_time).count();
    auto pose_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(post_pose_time - pre_pose_time).count();
    auto total_duration = detection_duration + pose_duration;

    // Output timing information
    if (OUTPUT_TIME) {
        // In nanoseconds
        std::cout << "Marker Detection Time: " << detection_duration << " ns" << std::endl;
        std::cout << "Pose Estimation Time: " << pose_duration << " ns" << std::endl;
        std::cout << "Total Time: " << total_duration << " ns" << std::endl;

        std::cout << std::endl;

        // In milliseconds
        std::cout << "Marker Detection Time: " << detection_duration / 1e6 << " ms" << std::endl;
        std::cout << "Pose Estimation Time: " << pose_duration / 1e6 << " ms" << std::endl;
        std::cout << "Total Time: " << total_duration / 1e6 << " ms" << std::endl;
    }

    // Write to CSV if path is provided
    if (!CSV_PATH.empty()) {
        // Check if we need to write the header
        // We need to check if the file exists and if it's empty
        bool write_header = false;
        std::ifstream infile(CSV_PATH);
        write_header = !infile.good() || infile.peek() == std::ifstream::traits_type::eof();
        infile.close();

        // Open the CSV file for writing
        std::ofstream csv_file;
        csv_file.open(CSV_PATH, std::ios::out | std::ios::app);
        if (csv_file.is_open()) {
                if (write_header) {
                    csv_file << "Marker Detection Time (ns),Pose Estimation Time (ns),Total Time (ns)\n";
                }
                csv_file << detection_duration << "," << pose_duration << "," << total_duration << "\n";
                csv_file.close();
                std::cout << "Timing information written to " << CSV_PATH << std::endl;
            } else {
                std::cerr << "Error: Could not open file " << CSV_PATH << " for writing." << std::endl;
            }
    }

    return 0;
}
