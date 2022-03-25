#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>


void onMouse(int event, int x, int y, int flags, void *userdata) {

    std::vector<Eigen::Vector3d>* points_clicked =
        static_cast<std::vector<Eigen::Vector3d>* >(userdata);
    if (cv::EVENT_LBUTTONUP == event) {
        Eigen::Vector3d pt; pt << x, y, 1;
        std::cout << "u" << pt << "\n";
        points_clicked->push_back(pt);
    }
}

std::vector<Eigen::Vector3d>
collectPoints(const cv::Mat& img) {
    cv::imshow("img", img);
    std::vector<Eigen::Vector3d> points_clicked;
    cv::setMouseCallback("img", onMouse, &points_clicked);
    std::cout << "Please click on 4 points\n";
    cv::waitKey(-1);
    return points_clicked;
}

Eigen::Matrix3d
findHomography(std::vector<Eigen::Vector3d> us,
               std::vector<Eigen::Vector3d> ups)
{
    Eigen::MatrixXd A(8, 9); A.setZero();
    for (int i = 0; i < us.size(); ++i) {
        A.block(2*i, 3, 1, 3) = -ups[i](2)*us[i].transpose();
        A.block(2*i, 6, 1, 3) = ups[i](1)*us[i].transpose();
        A.block(2*i, 0, 1, 3) = -ups[i](2)*us[i].transpose();
        A.block(2*i, 3, 1, 3) = ups[i](0)*us[i].transpose();
    }

    auto svd = A.jacobiSvd(Eigen::ComputeFullV);
    Eigen::Matrix3d H;
    Eigen::VectorXd nullspace = svd.matrixV().col(8);
    H.row(0) = nullspace.block(0, 0, 3, 1).transpose();
    H.row(1) = nullspace.block(3, 0, 3, 1).transpose();
    H.row(2) = nullspace.block(6, 0, 3, 1).transpose();
    return H;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Need input image path \n";
        return -1;
    }
    std::string fpath(argv[1]);
    std::cout << "file path : " << fpath << "\n";

    cv::Mat img = cv::imread(fpath);
    std::vector<Eigen::Vector3d> us= collectPoints(img);

    std::vector<Eigen::Vector3d> ups = us;
    ups[2](1) = ups[0](1);
    ups[3](1) = ups[1](1);
    for (int i = 0; i < us.size(); ++i) {
        std::cout << "u_" << i << " = " << us[i] << "\n";
        std::cout << "u'_" << i << " = " << ups[i] << "\n";
    }
    Eigen::Matrix3d H = findHomography(us, ups);
    std::cout << " H: " << H << "\n";
    return 0;
}
