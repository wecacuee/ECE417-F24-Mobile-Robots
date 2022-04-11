#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>


#include <iostream>

int main(int argc, char** argv) {

    std::string fpath;
    if (argc != 2) {
        std::cerr << "Need input image path \n";
    } else {
        fpath = argv[1];
    }

    std::cout << "file path : " << fpath << "\n";

    cv::Mat img = cv::imread(fpath, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "ERROR: Failed to load the image \n";
        return 1;
    }
    cv::namedWindow("windowname", cv::WINDOW_AUTOSIZE);
    cv::imshow("windowname", img);
    cv::waitKey(-1);

    cv::Size patternsize(10,8); //number of centers
    std::vector<cv::Point2f> centers; //this will be filled by the detected centers
    bool patternfound = cv::findChessboardCornersSB(
        img,
        patternsize,
        centers);

    Eigen::MatrixXd corners;
    cv::cv2eigen(cv::Mat(centers), corners);

    std::cout << "Corners: " << corners << "\n";
    if (! patternfound) {
        std::cerr << "Pattern not found \n";
        return -1;
    }

    std::cerr << "Drawing\n";
    cv::drawChessboardCorners(img,
                              patternsize,
                              cv::Mat(centers),
                              patternfound);

    cv::imshow("windowname", img);
    cv::waitKey(-1);
    return 0;
}
