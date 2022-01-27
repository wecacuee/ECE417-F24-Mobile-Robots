#define _USE_MATH_DEFINES
#include <cmath>
#include <ctime>

#include "Eigen/Dense"
#include <iostream>
#include <cstdlib>
#include <vector>

Eigen::Matrix3d random_rotation() {
    double x1 = rand() / double(RAND_MAX);
    double x2 = rand() / double(RAND_MAX);
    double x3 = rand() / double(RAND_MAX);
    double theta = 2*M_PI*x1; // Pick a rotation about the pole (z-axis)
    double phi = 2*M_PI*x2;   // Pick a direction to deflect the pole
    double z = x3;            // Pick the amount of pole deflection
    // Consider a vector for performing reflection
    Eigen::Matrix<double, 3, 1> V;
    V << cos(phi)*sqrt(z), sin(phi)*sqrt(z), sqrt(1-z);
    Eigen::Matrix<double, 3, 1> V2 = V;
    // Construct the rotation matrix around the Z-axis
    Eigen::Matrix3d Rtheta;
    Rtheta << cos(theta), sin(theta), 0,
    -sin(theta), cos(theta), 0,
    0, 0, 1;
    Eigen::Matrix3d I = Eigen::MatrixXd::Identity(3,3);
    // Rotate the Z-axis to a random orientation
    Eigen::Matrix3d VV = V2 * V.transpose();
    auto H = I - 2 * VV;
    return - H * Rtheta;
}

bool is_valid_rot(const Eigen::Matrix3d& R) {
    // TODO: Put a valid rotation check here.
    return 0;
}

Eigen::Matrix2d angle_to_rot2D(const double theta) {
    Eigen::Matrix2d R;
    R(0,0) = 1; // TODO: Replace with correct formula
    R(0,1) = 0; // TODO: Replace with correct formula
    R(1,0) = 0; // TODO: Replace with correct formula
    R(1,1) = 1; // TODO: Replace with correct formula
    return R;
}

Eigen::Matrix3d euler_to_rotmat(const double roll, 
                                const double pitch, 
                                const double yaw) {
    Eigen::Matrix3d Rx, Ry, Rz;
    Rx.setIdentity();
    Ry.setIdentity();
    Rz.setIdentity();

    Rx.block(1, 1, 2, 2) = angle_to_rot2D(0);   // TODO: Replace with correct formula
    Eigen::Matrix2d Rpitch = angle_to_rot2D(0); // TODO: Replace with correct formula
    Ry(2, 2) = 0; Ry(0, 0) = 0;
    Ry(2, 0) = 0; Ry(0, 2) = 0;
    Rz.block(0, 0, 2, 2) = angle_to_rot2D(0);   // TODO: Replace with correct formula
    return Rz * Ry * Rx;
}

Eigen::Vector3d rotmat_to_euler_rpy(const Eigen::Matrix3d& R) {
    Eigen::Vector3d rpy;
    rpy(0) = 0; // TODO: Replace with correct formula
    rpy(1) = 0; // TODO: Replace with correct formula
    rpy(2) = 0; // TODO: Replace with correct formula
    return rpy;
}

Eigen::Matrix4d transformation_matrix(const double roll, const double pitch, const double yaw,
                                      const Eigen::Vector3d& trans) {
    Eigen::Matrix4d T;
    Eigen::Matrix3d R = euler_to_rotmat(roll, pitch, yaw);
    T.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity(); // TODO : Replace this
    T.block(0, 3, 3, 1) = Eigen::Vector3d::Ones(); // TODO: Replace this
    return T;
}

int main(int argc, char* argv[]) {

    std::cout << "\nnChecking roll\n";
    {
      Eigen::Vector3d rpy;
      rpy << M_PI / 6, 0, 0;
      std::cout << "RPY:" << rpy << "\n";
      Eigen::Matrix3d Rrecovered = euler_to_rotmat(rpy(0), rpy(1), rpy(2));
      std::cout << "Rrecovered:" << Rrecovered << "\n";
      Eigen::Vector3d rpy_rec = rotmat_to_euler_rpy(Rrecovered);
      std::cout << "rpy_rec:" << rpy_rec << "\n";
      std::cout << "Is same?: " << (rpy_rec.isApprox(rpy)) << "\n";
    }

    std::cout << "\n\nChecking pitch\n";
    {
        Eigen::Vector3d rpy; rpy << 0, M_PI/6, 0;
      std::cout << "RPY:" << rpy << "\n";
      Eigen::Matrix3d Rrecovered = euler_to_rotmat(rpy(0), rpy(1), rpy(2));
      std::cout << "Rrecovered:" << Rrecovered << "\n";
      Eigen::Vector3d rpy_rec = rotmat_to_euler_rpy(Rrecovered);
      std::cout << "rpy_rec:" << rpy_rec << "\n";
      std::cout << "Is same?: " << (rpy_rec.isApprox(rpy)) << "\n";
    }

    std::cout << "\n\nChecking yaw\n";
    {
        Eigen::Vector3d rpy; rpy << 0, 0, M_PI/6;
      std::cout << "RPY:" << rpy << "\n";
      Eigen::Matrix3d Rrecovered = euler_to_rotmat(rpy(0), rpy(1), rpy(2));
      std::cout << "Rrecovered:" << Rrecovered << "\n";
      Eigen::Vector3d rpy_rec = rotmat_to_euler_rpy(Rrecovered);
      std::cout << "rpy_rec:" << rpy_rec << "\n";
      std::cout << "Is same?: " << (rpy_rec.isApprox(rpy)) << "\n";
    }


    std::cout << "\nnChecking random rotation 10 times\n";
    for (int i = 0; i < 10; i++) {
        Eigen::Matrix3d Rrand = random_rotation();
        std::cout << "Rrand:" << Rrand << "\n";
        std::cout << "is_valid:" << is_valid_rot(Rrand) << "\n";
        Eigen::Vector3d rpy = rotmat_to_euler_rpy(Rrand);
        std::cout << "RPY:" << rpy << "\n";
        Eigen::Matrix3d Rrecovered = euler_to_rotmat(rpy(0), rpy(1), rpy(2));
        std::cout << "Rrecovered:" << Rrecovered << "\n";
        Eigen::Vector3d rpy_rec = rotmat_to_euler_rpy(Rrecovered);
        std::cout << "rpy_rec:" << rpy_rec << "\n";
        std::cout << "Is same?: " << Rrand.isApprox(Rrecovered, 1e-5) << "\n";
    }

    std::cout << "\n\nChecking pitch = pi/2 \n";
    {
        Eigen::Vector3d rpy; rpy << M_PI/3, M_PI/2, M_PI/6;
      std::cout << "RPY:" << rpy << "\n";
      Eigen::Matrix3d Rrecovered = euler_to_rotmat(rpy(0), rpy(1), rpy(2));
      std::cout << "Rrecovered:" << Rrecovered << "\n";
      Eigen::Vector3d rpy_rec = rotmat_to_euler_rpy(Rrecovered);
      std::cout << "rpy_rec:" << rpy_rec << "\n";
      std::cout << "Is same?: " << (rpy_rec.isApprox(rpy)) << "\n";
    }

    std::cout << "\n\nChecking transformation matrix \n";
    {
        Eigen::Vector3d rpy; rpy << M_PI/3, M_PI/4, M_PI/6;
        Eigen::Vector3d trans; trans << 1, 1, 2;
        Eigen::Matrix4d T = transformation_matrix(rpy(0), rpy(1), rpy(2), trans);
        std::cout << "T: " << T << "\n";
    }
    return 0;
}
