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

bool
testHomographyFit(const std::vector<Eigen::Vector3d>& us,
                  const std::vector<Eigen::Vector3d>& ups,
                  const Eigen::Matrix3d& H)
{
    for (int i=0; i < us.size(); ++i) {
        Eigen::Vector3d ups_got = H * us[i];
        ups_got /= ups_got(2);
        if (ups_got.isApprox(ups[i])) {
            std::cout << "Good fit for " << i << "\n";
        } else {
            std::cout << "Bad fit for " << i << "\n";
            std::cout << "ups_got[" << i << "] = " << ups_got << "\n";
            std::cout << "ups[" << i << "] = " << ups[i] << "\n";
            return false;
        }
    }
    return true;
}

Eigen::Matrix3d
findHomography(const std::vector<Eigen::Vector3d>& us,
               const std::vector<Eigen::Vector3d>& ups)
{
    Eigen::MatrixXd A(8, 9); A.setZero();
    for (int i = 0; i < us.size(); ++i) {
        // [[0ᵀ      -w'ᵢ uᵢᵀ   yᵢ' uᵢᵀ]]
        //  [wᵢ'uᵢᵀ        0ᵀ   -xᵢ uᵢᵀ]]
        A.block(2*i, 3, 1, 3) = Eigen::Vector3d::Zero().transpose();// TODO Replace this with correct formula
        A.block(2*i, 6, 1, 3) = Eigen::Vector3d::Zero().transpose(); // TODO Replace this with correct formula 
        A.block(2*i+1, 0, 1, 3) = Eigen::Vector3d::Zero().transpose(); // TODO Replace this with correct formula
        A.block(2*i+1, 6, 1, 3) = Eigen::Vector3d::Zero().transpose(); // TODO Replace this with correct formula
    }

    auto svd = A.jacobiSvd(Eigen::ComputeFullV);
    // y = v₉
    Eigen::VectorXd nullspace = Eigen::VectorXd::Zero(9); // TODO Replace this with correct formula

    Eigen::Matrix3d H;
    H.row(0) = Eigen::Vector3d::Zero().transpose(); // TODO: replace with correct formula
    H.row(1) = Eigen::Vector3d::Zero().transpose(); // TODO: replace with correct formula
    H.row(2) = Eigen::Vector3d::Zero().transpose(); // TODO: replace with correct formula

    return H;
}

Eigen::MatrixXd
applyHomography(const Eigen::Matrix3d& H,
                const Eigen::MatrixXd& img) {
    Eigen::MatrixXd new_img(img.rows(), img.cols());
    Eigen::Vector3d u;
    Eigen::Vector3d up;
    for (int new_row = 0; new_row < new_img.rows(); ++new_row) {
        for (int new_col = 0; new_col < new_img.cols(); ++new_col) {
            u << new_col + 0.5, new_row + 0.5, 1;
            /**** Apply homography for each pixel ***/
            // u' = H * u
            up = Eigen::Vector3d::Zero(); // TODO replace with correct formula
            up /= up(2);
            /**** Apply homography for each pixel ***/
            int row = round(up(1));
            int col = round(up(0));
            if (0 <= row && row < img.rows()
                && 0 <= col && col < img.cols()) {
                new_img(new_row, new_col) = img(row, col);
            }
        }
    }
    return new_img;
}

void eigen_imshow(const Eigen::MatrixXd& eigen_new_img) {
    cv::Mat cv_new_img;
    cv::eigen2cv(eigen_new_img, cv_new_img);
    cv_new_img.convertTo(cv_new_img, CV_8U);
    cv::imshow("new_img", cv_new_img);
    cv::waitKey(-1);
}

int main(int argc, char** argv) {
    bool collect_points = false;
    std::string fpath;
    if (argc != 2) {
        // std::cerr << "Need input image path \n";
        fpath = "../../data/removing-perspective-distortion.png";
    } else {
        fpath = argv[1];
    }

    std::cout << "file path : " << fpath << "\n";

    cv::Mat img = cv::imread(fpath, cv::IMREAD_GRAYSCALE);
    cv::Mat img_64f;
    img.convertTo(img_64f, CV_64F);
    Eigen::MatrixXd eigen_img;
    cv::cv2eigen(img_64f, eigen_img);
    std::vector<Eigen::Vector3d> us;
    if (collect_points) {
        us = collectPoints(img);
    } else {
        {
            Eigen::Vector3d u;
            u << 253, 191, 1;
            us.push_back(u);
        }
        {
            Eigen::Vector3d u;
            u << 255, 233, 1;
            us.push_back(u);
        }
        {
            Eigen::Vector3d u;
            u << 329, 189, 1;
            us.push_back(u);
        }
        {
            Eigen::Vector3d u;
            u << 335, 225, 1;
            us.push_back(u);
        }
    }

    std::vector<Eigen::Vector3d> ups = us;
    ups[2](1) = ups[0](1);
    ups[3](1) = ups[1](1);
    for (int i = 0; i < us.size(); ++i) {
        std::cout << "u_" << i << " = " << us[i] << "\n";
        std::cout << "u'_" << i << " = " << ups[i] << "\n";
    }
    // Deliberately swapped ups and us because conversion from desired image pts 
    // to current image pts leads to a denser image.
    Eigen::Matrix3d H = findHomography(ups, us);
    std::cout << " H: " << H << "\n";
    testHomographyFit(us, ups, H);


    Eigen::MatrixXd eigen_new_img = applyHomography(H, eigen_img);
    Eigen::MatrixXd joined_img(std::max(eigen_img.rows(), eigen_new_img.rows()),
                               eigen_img.cols() + eigen_new_img.cols());
    joined_img.block(0, 0, eigen_img.rows(), eigen_img.cols()) = eigen_img;
    joined_img.block(0, eigen_img.cols(), eigen_new_img.rows(), eigen_new_img.cols()) =
        eigen_new_img;
    eigen_imshow(joined_img);
    return 0;
}
