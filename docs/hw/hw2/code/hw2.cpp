#define _USE_MATH_DEFINES
#include <cmath>
#include <ctime>
#include <cassert>

#include "Eigen/Dense"
#include <iostream>
#include <cstdlib>
#include <vector>
#include <utility> // for std::pair

/**************************************************************************/
/*************************** Homework 1************************************/
/**************************************************************************/

double drand() {
    return rand() / double(RAND_MAX);
}

Eigen::Matrix3d random_rotation() {
    double x1 = drand();
    double x2 = drand();
    double x3 = drand();
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
    return (R * R.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-5) &&
        abs(R.determinant() - 1) < 1e-5;
}

Eigen::Matrix2d angle_to_rot2D(const double theta) {
    Eigen::Matrix2d R;
    R(0,0) = cos(theta); // TODO: Replace with correct formula
    R(0,1) = -sin(theta); // TODO: Replace with correct formula
    R(1,0) = sin(theta); // TODO: Replace with correct formula
    R(1,1) = cos(theta); // TODO: Replace with correct formula
    return R;
}

/**************************************************************************/
/*************************** Homework 2 ************************************/
/**************************************************************************/


// Converts a given vector into a cross product matrix
Eigen::Matrix3d cross_product_matrix(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d K;
   // TODO: Fill in correct formula for K
    K << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    return K;
}

// Converts axis-angle representation to a rotation matrix
Eigen::Matrix3d axis_angle_to_rotmat(const double theta, const Eigen::Vector3d&
axis) {
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Vector3d axisunitvector = axis / axis.norm();
    Eigen::Matrix3d K = I;// TODO: Replace this
    return I; // TODO: Replace this
}

// Converts a rotation matrix to axis-angle representation
std::pair<double, Eigen::Vector3d>
rotmat_to_axis_angle(const Eigen::Matrix3d& R)
{
    double prec = 1e-6; // precision
    double theta = 0; // TODO: Replace with correct formula. You can use R.trace() for finding the trace.
    Eigen::Vector3d axis;
    if (0) { // TODO: Replace with correct condition
        axis << 1, 0, 0; // TODO: Replace with correct formula
    } else {
        axis << 1, 0, 0; // TODO: Replace with correct formula
        axis /= 1; // TODO: Replace with correct formula

    }
    return std::make_pair(theta, axis);
}

// Checks if Axis angle is the same
bool is_same_axis_angle(
    const double theta_a, const Eigen::Vector3d& axis_a,
    const double theta_b, const Eigen::Vector3d& axis_b) {
    return (abs(theta_a - theta_b) < 1e-6) && axis_a.isApprox(axis_b);
}

int main(int argc, char* argv[]) {
    srand(1);

    std::cout << "\n\nChecking roll\n";
    {
      Eigen::Vector3d axis;
      axis << 1, 0, 0;
      double theta = M_PI / 6;
      std::cout << "axis:" << axis << "; theta: " << theta << "\n";
      Eigen::Matrix3d R = axis_angle_to_rotmat(theta, axis);
      std::cout << "R:" << R << "\n";
      std::cout << "Is valid R: " <<  is_valid_rot(R) << "\n";
      // std::cout << "Is same as RPY " << R.isApprox(euler_to_rotmat(M_PI / 6, 0, 0)) << "\n";

      Eigen::Vector3d axis_recovered;
      double theta_recovered;
      std::tie(theta_recovered, axis_recovered) = rotmat_to_axis_angle(R);
      std::cout << "axis_recovered:" << axis_recovered << "; theta_recovered: " << theta_recovered << "\n";
      std::cout << "Is same?: " << is_same_axis_angle(theta, axis,
                                                      theta_recovered, axis_recovered) << "\n";
    }

    std::cout << "\n\nChecking pitch\n";
    {
        Eigen::Vector3d axis;
        axis << 0, 1, 0;
        double theta = M_PI / 6;
        std::cout << "axis:" << axis << "; theta: " << theta << "\n";
        Eigen::Matrix3d R = axis_angle_to_rotmat(theta, axis);
        std::cout << "R:" << R << "\n";
        std::cout << "Is valid R: " <<  is_valid_rot(R) << "\n";
        // std::cout << "Is same as RPY " << R.isApprox(euler_to_rotmat(0, M_PI / 6, 0)) << "\n";

        Eigen::Vector3d axis_recovered;
        double theta_recovered;
        std::tie(theta_recovered, axis_recovered) = rotmat_to_axis_angle(R);
        std::cout << "axis_recovered:" << axis_recovered << "; theta_recovered: " << theta_recovered << "\n";
        std::cout << "Is same?: " << is_same_axis_angle(theta, axis,
                                                        theta_recovered, axis_recovered) << "\n";
    }

    std::cout << "\n\nChecking yaw\n";
    {
        Eigen::Vector3d axis;
        axis << 0, 0, 1;
        double theta = M_PI / 6;
        std::cout << "axis:" << axis << "; theta: " << theta << "\n";
        Eigen::Matrix3d R = axis_angle_to_rotmat(theta, axis);
        std::cout << "R:" << R << "\n";
        std::cout << "Is valid R: " <<  is_valid_rot(R) << "\n";
        // std::cout << "Is same as RPY: " << R.isApprox(euler_to_rotmat(0, 0, M_PI/2)) << "\n";

        Eigen::Vector3d axis_recovered;
        double theta_recovered;
        std::tie(theta_recovered, axis_recovered) = rotmat_to_axis_angle(R);
        std::cout << "axis_recovered:" << axis_recovered << "; theta_recovered: " << theta_recovered << "\n";
        std::cout << "Is same?: " << is_same_axis_angle(theta, axis,
                                                        theta_recovered, axis_recovered) << "\n";
    }


    std::cout << "\n\nChecking random rotation 10 times\n";
    for (int i = 0; i < 10; i++) {
        Eigen::Matrix3d Rrand = (drand() > 0.5) ? random_rotation() : Eigen::Matrix3d::Random();
        std::cout << "Rrand:" << Rrand << "\n";
        std::cout << "is_valid:" << is_valid_rot(Rrand) << "\n";
        Eigen::Vector3d axis;
        double theta;
        std::tie(theta, axis) = rotmat_to_axis_angle(Rrand);
        std::cout << "axis:" << axis << "; theta: " << theta << "\n";
        Eigen::Matrix3d Rrecovered = axis_angle_to_rotmat(theta, axis);
        std::cout << "Rrecovered:" << Rrecovered << "\n";
        std::cout << "Is same?: " << Rrand.isApprox(Rrecovered, 1e-5) << "\n";
    }

    std::cout << "\n\nChecking theta = pi \n";
    {
        Eigen::Vector3d axis = Eigen::Vector3d::Random();
        double theta = M_PI;
        std::cout << "axis:" << axis << "; theta: " << theta << "\n";
        Eigen::Matrix3d R = axis_angle_to_rotmat(theta, axis);
        std::cout << "R:" << R << "\n";
        Eigen::Vector3d axis_recovered;
        double theta_recovered;
        std::tie(theta_recovered, axis_recovered) = rotmat_to_axis_angle(R);
        std::cout << "axis_recovered:" << axis_recovered << "; theta_recovered: " << theta_recovered << "\n";
        // std::cout << "Is same?: " << is_same_axis_angle(theta, axis,
        //                                                 theta_recovered, axis_recovered) << "\n";

        // std::cout << "Is same flipped?: " << is_same_axis_angle(
        //     theta, axis,
        //     theta_recovered, -1 * axis_recovered) << "\n";
    }
    return 0;
}
