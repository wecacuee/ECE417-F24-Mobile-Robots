#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>


#include <iostream>


// Find two chessboard corners with sliding window approach
std::vector<Eigen::MatrixXf>
findAllChessboardCorners(cv::Mat img_gray,
                         cv::Size_<double> window_size_ratio = cv::Size_<double>(0.5, 0.85),
                         cv::Size_<double> step_size_ratio  = cv::Size_<double>(0.1, 0.1),
                         cv::Size patternsize = cv::Size(8,6) //number of centers
    )
{
    int window_size_rows = static_cast<int>(
        img_gray.rows * window_size_ratio.height);
    int window_size_cols = static_cast<int>(
        img_gray.cols * window_size_ratio.width);

    int step_size_rows = static_cast<int>(
        img_gray.rows * step_size_ratio.height);
    int step_size_cols = static_cast<int>(
        img_gray.cols * step_size_ratio.width);

    // Sliding window loop
    std::vector<Eigen::MatrixXf> chessboards;
    for (int r = 0; r + window_size_rows < img_gray.rows; r += step_size_rows) {
        for (int c = 0; c + window_size_cols < img_gray.cols; c += step_size_cols) {

            Eigen::Vector2f offset(c, r);

            // Get a window and see if you get a detection
            cv::Mat window_img = img_gray(
                cv::Range(r, r + window_size_rows),
                cv::Range(c, c + window_size_cols));

            std::vector<cv::Point2f> corners_cvpt; //this will be filled by the detected centers
            bool patternfound = findChessboardCornersSB(
                window_img, patternsize, corners_cvpt);

            cv::imshow("windowname", window_img);
            cv::waitKey(-1);

            if (patternfound) {
                // Chessboard found

                {
                    cv::Mat window_img_bgr;
                    cv::cvtColor(window_img, window_img_bgr, cv::COLOR_BGR2GRAY);
                    cv::drawChessboardCorners(window_img_bgr,
                                              patternsize,
                                              cv::Mat(corners_cvpt),
                                              patternfound);

                    cv::imshow("windowname", window_img_bgr);
                    cv::waitKey(-1);
                }
                cv::Mat corners_cvmat = cv::Mat( corners_cvpt ).reshape(1);

                Eigen::MatrixXf corners;
                cv::cv2eigen(corners_cvmat, corners);
                // Adds offset to all the rows of detected corners
                corners.rowwise() += offset.transpose();

                // If pattern already detected then do not
                // add it to the list
                bool pattern_already_detected = false;
                for (const Eigen::MatrixXf& prev_corners : chessboards)  {
                    if (prev_corners.isApprox(corners, 1e-3)) {
                        pattern_already_detected = true;
                        std::cout << "Pattern was already detected earlier \n";
                    }
                }

                if (  ! pattern_already_detected) {
                    chessboards.push_back(corners);

                    std::cerr << "Drawing corners on the image\n";
                    cv::Mat corners_mat;
                    cv::eigen2cv(corners, corners_mat);
                    cv::Mat img_bgr;
                    cv::cvtColor(img_gray, img_bgr, cv::COLOR_BGR2GRAY);
                    cv::drawChessboardCorners(img_bgr,
                                              patternsize,
                                              corners_mat,
                                              patternfound);

                    cv::imshow("windowname", img_bgr);
                    cv::waitKey(-1);
                }
            }
        }
    }
    std::cout << "Number  of chessboards detected = " << chessboards.size() << "\n";
    return chessboards;
}

int main(int argc, char** argv) {

    std::string fpath;
    if (argc != 2) {
        std::cerr << "Need input image path \n";
    } else {
        fpath = argv[1];
    }

    std::cout << "file path : " << fpath << "\n";

    cv::Mat img_bgr = cv::imread(fpath);
    if (img_bgr.empty()) {
        std::cerr << "ERROR: Failed to load the image \n";
        return 1;
    }
    cv::namedWindow("windowname", cv::WINDOW_AUTOSIZE);
    cv::imshow("windowname", img_bgr);
    cv::waitKey(-1);

    std::vector<Eigen::MatrixXf> chessboards = findAllChessboardCorners(img_bgr);

    return 0;
}
