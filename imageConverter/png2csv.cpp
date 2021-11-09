#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/imgproc.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include <iostream>
#include <fstream>
// #include "opencv4/opencv2/core/eigen.hpp"



int main(){
    cv::Mat img = cv::imread("parkingslots.png", 0);
    cv::Mat mapVisualizer = cv::Mat(cv::Size(1075,701), CV_8UC1, cv::Scalar(255));
    cv::Mat mask;
    compare(img, cv::Scalar::all(207), mask, cv::CMP_GT);
    mapVisualizer.setTo(cv::Scalar::all(0), mask);

    cv::Mat sizeDown;
    cv::resize(mapVisualizer, sizeDown, cv::Size(108, 70), cv::INTER_LINEAR);

    if (img.empty()) 
        {
        std::cout << "Could not open or find the image" << std::endl;
        std::cin.get(); //wait for any key press
        return -1;
        }
    cv::imshow("output", sizeDown);
    cv::waitKey(0);

    cv::Mat inverted;
    cv::Mat grayscale;
    cv::threshold(sizeDown, grayscale, 100, 255, cv::THRESH_BINARY);
    cv::bitwise_not(grayscale, inverted);

    // compare(inverted, cv::Scalar::all(207), mask, cv::CMP_GT);
    // inverted.setTo(cv::Scalar::all(1), mask);



    std::ofstream outputFile("costmap_carla.csv");
    outputFile << format(inverted, cv::Formatter::FMT_CSV) << std::endl;
    outputFile.close();
    cv::destroyWindow("output");
    return 0;
}