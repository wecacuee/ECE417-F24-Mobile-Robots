#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <iostream>
#include <cstdlib>

// sudo apt install libeigen3-dev ros-noetic-cv-bridge
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>


#include <iostream>


void
showChessboardCorners(const cv::Mat window_img,
                      const cv::Mat corners_mat,
                      const cv::Size patternsize,
                      const bool patternfound)
{
    // cv::Mat window_img_bgr;
    // cv::cvtColor(window_img, window_img_bgr, cv::COLOR_BGR2GRAY);
        cv::drawChessboardCorners(window_img,
                                  patternsize,
                                  corners_mat,
                                  patternfound);

        cv::imshow("windowname", window_img);
        cv::waitKey(1000); // wait for 1 sec
}


// Find two chessboard corners with sliding window approach
std::vector<Eigen::MatrixXf>
findAllChessboardCorners(const cv::Mat img_bgr,
                         const cv::Size patternsize,  // number of centers
                         // TODO: This window_size_ratio may or may not work for your chessboard
                         // This should the percentage width x height of the image that
                         // selects only one chessboard of the two chessboards.
                         const cv::Size_<double> window_size_ratio = cv::Size_<double>(0.4, 0.8),
                         const cv::Size_<double> step_size_ratio  = cv::Size_<double>(0.1, 0.1)
    )
{
    int window_size_rows = static_cast<int>(
        img_bgr.rows * window_size_ratio.height);
    int window_size_cols = static_cast<int>(
        img_bgr.cols * window_size_ratio.width);

    int step_size_rows = static_cast<int>(
        img_bgr.rows * step_size_ratio.height);
    int step_size_cols = static_cast<int>(
        img_bgr.cols * step_size_ratio.width);

    // Sliding window loop
    std::vector<Eigen::MatrixXf> chessboards;
    for (int r = 0; r + window_size_rows < img_bgr.rows; r += step_size_rows) {
        for (int c = 0; c + window_size_cols < img_bgr.cols; c += step_size_cols) {

            Eigen::Vector2f offset(c, r);

            // Get a window and see if you get a detection
            cv::Mat window_img = img_bgr(
                cv::Range(r, r + window_size_rows),
                cv::Range(c, c + window_size_cols));

            std::vector<cv::Point2f> corners_cvpt; //this will be filled by the detected centers
            bool patternfound = findChessboardCornersSB(
                window_img, patternsize, corners_cvpt);

            cv::imshow("windowname", window_img);
            cv::waitKey(100); // wait for 1 sec

            if (patternfound) {
                // Chessboard found
                showChessboardCorners(window_img,
                                      cv::Mat(corners_cvpt),
                                      patternsize,
                                      patternfound);

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
                    showChessboardCorners(img_bgr,
                                          corners_mat,
                                          patternsize,
                                          patternfound);
                }
            }
        }
    }
    std::cout << "Number  of chessboards detected = " << chessboards.size() << "\n";
    return chessboards;
}

bool
testProjectionFit(const std::vector<Eigen::VectorXd>& Xs,
                  const std::vector<Eigen::VectorXd>& us,
                  const Eigen::MatrixXd& P)
{
    for (int i=0; i < us.size(); ++i) {
        Eigen::Vector3d us_got = P * Xs[i];
        us_got /= us_got(2);
        if (us_got.isApprox(us[i])) {
            std::cout << "Good fit for " << i << "\n";
        } else {
            std::cout << "Bad fit for " << i << "\n";
            std::cout << "ups_got[" << i << "] = " << us_got << "\n";
            std::cout << "ups[" << i << "] = " << us[i] << "\n";
            return false;
        }
    }
    return true;
}

Eigen::MatrixXd
findProjection(const Eigen::MatrixXd& Xs,
               const Eigen::MatrixXd& us)
{
    if (Xs.rows() < 6 || us.rows() < 6) {
        std::cerr << "Need at least six points got " << Xs.rows() << " and " << us.rows() << "\n";
        throw std::runtime_error("Need atleast six points");
    }
    int DX =  Xs.cols(); // Dimension of X = 4
    int Du =  us.cols(); // Dimension of u = 3
    Eigen::MatrixXd A(2*us.rows(), DX*Du); A.setZero();
    for (int i = 0; i < us.rows(); ++i) {
        // [[0ᵀ      -w'ᵢ Xᵢᵀ   yᵢ' Xᵢᵀ]]
        //  [wᵢ'Xᵢᵀ        0ᵀ   -xᵢ Xᵢᵀ]]
        // TODO: Initialize two rows of the A matrix properly
        // A.block(2*i,   DX,   1, DX) = ?
        // A.block(2*i,   DX*2, 1, DX) = ?
        // A.block(2*i+1, 0,    1, DX) = ?
        // A.block(2*i+1, DX*2, 1, DX) = ?
    }

    auto svd = A.jacobiSvd(Eigen::ComputeFullV);
    // y = v₁₂
    // TODO: Find the nullspace using SVD
    // Eigen::VectorXd nullspace = ?

    Eigen::MatrixXd P(Du, DX);
    // TODO: Find the P matrix from nullspace
    // P.row(0) = ?
    // P.row(1) = ?
    // P.row(2) = ?
    return P;
}

Eigen::MatrixXd
generateChessboard3DCoordinates(const cv::Size& patternsize,
                              double chessboxdim = 0.024 // meters
    )
{
    int corner_idx = 0;
    Eigen::MatrixXd corners(patternsize.height*patternsize.width*2,
                            4);
    // Let left chessboard be in YZ plane
    for (int z = 0;  z  < patternsize.height; ++z) {
        for (int y  = 0; y  < patternsize.width; ++y) {
            // TODO: Initialize the 3D Chessboard corners properly, then uncomment these lines
            // corners(corner_idx, 0) =  ?
            // corners(corner_idx, 1) =  ?
            // corners(corner_idx, 2) =  ?
            // corners(corner_idx, 3) =  ?
            ++corner_idx;
        }
    }

    // Let right  chessboard be in  the XZ plane
    for (int z = 0;  z  < patternsize.height; ++z) {
        for (int x  = 0; x  < patternsize.width; ++x) {
            // TODO: Initialize the 3D Chessboard corners properly, then uncomment these lines
            // corners(corner_idx, 0) =  ?
            // corners(corner_idx, 1) =  ?
            // corners(corner_idx, 2) =  ?
            // corners(corner_idx, 3) =  ?
            ++corner_idx;
        }
    }
    return corners;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
gramSchmidtForRQFactorization(const Eigen::MatrixXd& M) {
    assert(M.rows() == M.cols()); // Throw error if M.rows() ≠ M.cols()

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(M.rows(), M.cols());
    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();

    // TODO: Do the Gram Schmidt process to find K and R
    return std::make_pair(K, R);
}


void eigen_imshow(const Eigen::MatrixXd& eigen_new_img) {
    cv::Mat cv_new_img;
    cv::eigen2cv(eigen_new_img, cv_new_img);
    cv_new_img.convertTo(cv_new_img, CV_8U);
    cv::imshow("new_img", cv_new_img);
    cv::waitKey(1000); // wait for 1 sec
}

/**
 * This functions contains the main logic of camera calibration using 3D chessboards
 */
bool
findAndPrintCameraMatrix(cv::Mat img_bgr) {
    cv::Size patternsize(6, 8);
    std::vector<Eigen::MatrixXf> chessboards = findAllChessboardCorners(
        img_bgr, patternsize);
    if ( chessboards.size() < 2)
        return false;

    Eigen::MatrixXd us(chessboards[0].rows() + chessboards[1].rows(),
                       3);
    us.setOnes();
    us.block(0, 0, chessboards[0].rows(), 2) = chessboards[0].cast<double>();
    us.block(chessboards[0].rows(), 0, chessboards[0].rows(), 2) = chessboards[1].cast<double>();

    Eigen::MatrixXd Xs = generateChessboard3DCoordinates(patternsize);
    Eigen::MatrixXd P = findProjection(Xs, us);
    std::cout << "P : " << P << "\n";

    Eigen::Matrix3d K;
    Eigen::Matrix3d R;
    std::tie(K, R) = gramSchmidtForRQFactorization(P.block(0, 0, 3, 3));
    K = K / K(2,2);
    std::cout << "K : " << K << "\n";
    std::cout << "R : " << R << "\n";
    std::cout << "t : " << K.inverse() * P.block(0, 3, 3, 1) << "\n";
    return true;
}

// Change this to main if you want to read from image
int main_to_read_from_image(int argc, char** argv) {

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
    cv::waitKey(1000);
    // cv::Mat img_gray;
    // cv::cvtColor(img_bgr, img_gray, cv::COLOR_BGR2GRAY);
    cv::Mat img_gray = img_bgr;

    bool success = findAndPrintCameraMatrix(img_gray);

    return 0;
}

void imageCallback(
    const sensor_msgs::Image::ConstPtr& msg) {

    // http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
    cv_bridge::CvImageConstPtr cv_image_ptr;
    try {
        cv_image_ptr = cv_bridge::toCvShare(
            msg,
            sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
        ROS_FATAL("cv_bridge exception: %s", e.what());
        return;
    }

    bool success = findAndPrintCameraMatrix(cv_image_ptr->image);
    if (success)
        ros::shutdown();
}

int main_ros(int argc, char** argv) {
    ros::init(argc, argv, "camera_calibration");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cv_camera/image_raw", 1, &imageCallback);

    ros::spin();
    return 0;
}

int main(int argc, char** argv) {
    // // Uncomment the following line if you want to read from a single image.
    // // Pass image path as an argument to the command
    // // For example,
    // // rosrun camera_calibration_3d camera_calibration src/camera_calibration_3d/data/two-chessboard.jpg
    // return main_to_read_from_image(argc, argv);


    // Calls the main function to ros
    return main_ros(argc, argv);
}
