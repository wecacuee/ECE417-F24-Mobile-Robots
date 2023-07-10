#include <cstdlib>
#include <iostream>
#include <vector>

#include <Eigen/Dense>

// #include "matplotlibcpp.h"
#include "Eigen/SVD"

// namespace plt = matplotlibcpp;

/**
 * Converts Eigen VectorXd to std::vector
 *
 * Take arguments by value so that we can create a contiguous data copy. Eigen
 * 3.4 supports stl containers, this wont be necessary.
 */
std::vector<double>
to_std_vector(const Eigen::VectorXd v) {
    std::vector<double>  stdv(v.data(), v.data() + v.rows() * v.cols());
    return stdv;
}

Eigen::Vector3d
fit_line_to_points() {

    // Initialize a 4 x 3 matrix
    Eigen::MatrixXd A(4, 3);
    A << 100, 98, 1,
         105, 95, 1,
         107, 90, 1,
         110, 85, 1;

    // Compute the SVD by any algorithm. Here we use Jacobi algorithm.
    Eigen::JacobiSVD<Eigen::MatrixXd > svd =  A.jacobiSvd(
        Eigen::ComputeFullU | Eigen::ComputeFullV );
    std::cout << "\nU = " << svd.matrixU() << "\n";
    std::cout << "\ndiag(𝛴) = " << svd.singularValues() << "\n";
    std::cout << "\nV = " << svd.matrixV() << "\n";


    Eigen::MatrixXd V = svd.matrixV();
    // We take the rank of the matrix to be 2 and take the last column of V to
    // be the basis of nullspace
    Eigen::Vector3d l = V.col(2);
    std::cout << "\nRepr of line = Basis of NullSpace(A) = " <<  l << "\n";

    // plt::named_plot("points", to_std_vector(A.col(0)), to_std_vector(A.col(1)), "r+");

    Eigen::VectorXd xs = A.col(0);
    // equation of line is l(0)x + l(1)y + l(2) = 0
    // y = -(l(2) + l(0)x) / l(1)
    // Eigen::VectorXd ys = -( l(2) + l(0) * xs.array()) / l(1);
    // plt::named_plot("line", to_std_vector(xs), to_std_vector(ys), "b-");
    // plt::show();

    return l;
}

double drand(){
    return rand() / double(RAND_MAX);
}

Eigen::Vector4d
image_line_to_plane_3D(const Eigen::Matrix3d& K,
                       const Eigen::Vector3d& l)
{
    // lᵀKX = 0
    // (Kᵀl)X = 0
    // pᵀ X = 0
    // p = ?
    Eigen::Vector4d p;  p.setZero();
    p.topRows(3) = K.transpose() * l;
    return p;
}

Eigen::Vector4d plane_from_camera_to_world(
    const Eigen::Matrix3d& wRc,
    const Eigen::Vector3d& wtc,
    const Eigen::Vector4d& plane_c) {
    // ᶜT_w = [[ʷR_cᵀ,  - ʷR_cᵀ ʷt_c],
    //         [0    ,            1]]
    Eigen::Matrix4d cTw; cTw.setIdentity();
    cTw.topLeftCorner(3, 3) = wRc.transpose();
    cTw.topRightCorner(3, 1) = - wRc.transpose() * wtc;


    // pᵀ X_c = 0
    // pᵀ (ᶜT_w X_w) = 0
    // (ᶜT_wᵀp)ᵀ X_w = 0

    return cTw.transpose() * plane_c;
}


// Converts a given vector into a cross product matrix
Eigen::Matrix3d cross_product_matrix(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d K;
    K << 0.0 , -vec(2), vec(1),
        vec(2), 0.0, -vec(0),
        -vec(1), vec(0), 0.0;
    return K;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
plane_plane_intersection(
    const Eigen::Vector4d& plane_p,
    const Eigen::Vector4d& plane_q)
{
    // pₚᵀ x = 0
    // p_qᵀ x = 0
    Eigen::Vector3d direction = cross_product_matrix(plane_p.topRows(3)) * plane_q.topRows(3);

    // A = [ pₚᵀ(1:3)
    //      p_qᵀ(1:3)]
    // b = [ -pₚ(4)
    //       -p_q(4)]
    Eigen::MatrixXd A(2, 3);
    A.row(0) = plane_p.topRows(3).transpose();
    A.row(1) = plane_q.topRows(3).transpose();

    Eigen::Vector2d b;
    b(0) = - plane_p(3);
    b(1) = - plane_q(3);

    Eigen::MatrixXd A_pseudo_inv = A.transpose()*(A*A.transpose()).inverse();
    Eigen::Vector3d point_on_line = A_pseudo_inv * b;

    return std::make_pair(direction, point_on_line);
}


std::pair<Eigen::Vector3d, Eigen::Vector3d>
find_image_line_to_plane_3D_intersection(const Eigen::Vector3d& line,
                                         const Eigen::Vector4d& road_plane) {
    Eigen::Matrix3d K;
    K << 100, 0, 100,
        0, 100, 50,
        0, 0, 1;

    Eigen::Matrix3d wRc;
    wRc << 1, 0, 0,
        0, 0, 1,
        0, -1, 0;

    Eigen::Vector3d wtc;
    wtc << 0, 0, 1;

    // Step 1: convert image line to plane 3d
    Eigen::Vector4d plane_c = image_line_to_plane_3D(K, line);

    // Step 2: convert plane in camera coordinates to world coordinates
    Eigen::Vector4d plane_w = plane_from_camera_to_world(wRc, wtc, plane_c);

    // Step 3: Find plane-plane intersection
    return plane_plane_intersection(plane_w, road_plane);
}

bool
test_by_random_points(const Eigen::Vector3d& line,
                      const Eigen::Vector4d& road_plane,
                      const Eigen::Vector3d& direction,
                      const Eigen::Vector3d& point_on_line) {

    Eigen::Matrix3d K;
    K << 100, 0, 100,
        0, 100, 50,
        0, 0, 1;

    Eigen::Matrix3d wRc;
    wRc << 1, 0, 0,
        0, 0, 1,
        0, -1, 0;

    Eigen::Vector3d wtc;
    wtc << 0, 0, 1;
    double lambda = drand();
    Eigen::Vector3d rand_point_on_line = lambda * direction + point_on_line;
    bool is_point_on_road_plane = abs(road_plane.topRows(3).transpose() * rand_point_on_line + road_plane(3)) < 1e-6;

    Eigen::Vector3d X_c = wRc.transpose() * rand_point_on_line - wRc.transpose() * wtc;
    Eigen::Vector3d u = K * X_c;
    bool is_point_on_line = abs(line.transpose() * u) < 1e-6;
    return is_point_on_line && is_point_on_road_plane;
}


int main(int argc, char** argv) {
    {
      std::cout << "\n\nTesting for problem 8 values\n";
      Eigen::Vector3d l;
      l << 1, -1, -150;

      Eigen::Vector4d q;
      q << 0, 100, -1, 0;

      Eigen::Vector3d direction;
      Eigen::Vector3d point_on_line;
      std::tie(direction, point_on_line) = find_image_line_to_plane_3D_intersection(l, q);
      std::cout << "direction: " << direction << "; point_on_line: " << point_on_line << "\n";
      std::cout << "Test by random point: " << (test_by_random_points(l, q, direction,
                                                                      point_on_line) ? "succeeded" : "failed") << "\n";
    }

    {
        std::cout << "\n\nTesting for fitline.cpp values\n";
        Eigen::Vector3d l = fit_line_to_points();

        Eigen::Vector4d q;
        q << 0, 100, -1, 0;

        Eigen::Vector3d direction;
        Eigen::Vector3d point_on_line;
        std::tie(direction, point_on_line) = find_image_line_to_plane_3D_intersection(l, q);
        std::cout << "direction: " << direction << "; point_on_line: " << point_on_line << "\n";
        std::cout << "Test by random point: " << (test_by_random_points(l, q, direction,
                                                                        point_on_line) ? "succeeded" : "failed") << "\n";
    }

    std::cout << "\n\nTesting for random values\n";
    for (int i = 0; i < 10; ++i) {
        std::cout << "\n\nTesting for random value iteration: " << i << "\n";
        Eigen::Vector3d l = Eigen::Vector3d::Random();

        Eigen::Vector4d q = Eigen::Vector4d::Random();
        std::cout << "Input line: " << l << "; Road plane: " << q << "\n";

        Eigen::Vector3d direction;
        Eigen::Vector3d point_on_line;
        std::tie(direction, point_on_line) = find_image_line_to_plane_3D_intersection(l, q);
        std::cout << "direction: " << direction << ";\n point_on_line: " << point_on_line << "\n";
        std::cout << "Test by random point: " << (test_by_random_points(l, q, direction,
                                                                        point_on_line) ? "succeeded" : "failed") << "\n";
    }

    return 0;
}
