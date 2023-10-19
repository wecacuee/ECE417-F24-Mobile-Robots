#include <iostream>
#include <Eigen/Dense>

Eigen::Vector4d
image_line_to_plane_3D(const Eigen::Matrix3d& K,
                       const Eigen::Vector3d& l)
{
    // lᵀKX = 0
    // (Kᵀl)X = 0
    // pᵀ X = 0
    // p = ?
    Eigen::Vector4d p;  p.setZero();
    // p.topRows(3) = TODO fill correct formula here, then uncomment this line
    return p;
}

Eigen::Vector4d plane_from_camera_to_world(
    const Eigen::Matrix3d& wRc,
    const Eigen::Vector3d& wtc,
    const Eigen::Vector4d& plane_c) {
    // ᶜT_w = [[ʷR_cᵀ,  - ʷR_cᵀ ʷt_c],
    //         [0    ,            1]]
    Eigen::Matrix4d cTw; cTw.setZero();
    // cTw.topLeftCorner(3, 3) = TODO fill correct formula here, then uncomment this line
    // cTw.topRightCorner(3, 1) = TODO fill correct formula here, then uncomment this line


    // pᵀ X_c = 0
    // pᵀ (ᶜT_w X_w) = 0
    // (ᶜT_wᵀp)ᵀ X_w = 0

    // return TODO
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
    // Eigen::Vector3d direction = TODO fill correct formula here, then uncomment this line

    // A = [ pₚᵀ(1:3)
    //      p_qᵀ(1:3)]
    // b = [ -pₚ(4)
    //       -p_q(4)]
    Eigen::MatrixXd A(2, 3);
    // A.row(0) = TODO fill correct formula here, then uncomment this line
    // A.row(1) = TODO fill correct formula here, then uncomment this line

    Eigen::Vector2d b;
    // b(0) = TODO fill correct formula here, then uncomment this line
    // b(1) = TODO fill correct formula here, then uncomment this line

    // Eigen::MatrixXd A_pseudo_inv = TODO fill correct formula here, then uncomment this line
    // Eigen::Vector3d point_on_line = TODO fill correct formula here, then uncomment this line

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
    }

    return 0;
}
