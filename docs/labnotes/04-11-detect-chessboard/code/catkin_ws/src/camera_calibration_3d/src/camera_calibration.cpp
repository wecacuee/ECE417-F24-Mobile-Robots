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
    cv::Mat img_corners = cv::imread(fpath);
    if (img.empty()) {
        std::cerr << "ERROR: Failed to load the image \n";
        return 1;
    }
    cv::namedWindow("windowname", cv::WINDOW_AUTOSIZE);
    cv::imshow("windowname", img);
    cv::waitKey(-1);

    cv::Size patternsize(8,6); //number of centers
    std::vector<cv::Point2f> centers; //this will be filled by the detected centers
    bool patternfound = cv::findChessboardCornersSB(
        img,
        patternsize,
        centers);
    if (patternfound) {
        std::cout << "Pattern found \n";
    } else {
        std::cerr << "Pattern not found \n";
        return -1;
    }

    cv::Mat centers_mat ( centers );

    std::cerr << "Drawing\n";
    cv::drawChessboardCorners(img_corners,
                              patternsize,
                              centers_mat,
                              patternfound);

    cv::imshow("windowname", img_corners);
    cv::waitKey(-1);


    Eigen::MatrixXd corners(centers.size(), 2);
    for (int i = 0; i < centers.size(); ++i) {
        corners(i, 0) = centers[i].x;
        corners(i, 1) = centers[i].x;
    }
    std::cout << "Corners : " << corners << "\n";
    return 0;
}
