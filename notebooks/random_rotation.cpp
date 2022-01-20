// Algorithm to generate 3x3 random rotation matrix
// 
// https://archive.org/details/isbn_9780124096738/page/117/mode/2up "Fast Random Rotation matrics" James Arvo
// To compile use:
// > sudo apt install libeigen3-dev
// > g++ -I/usr/include/eigen3 random_rotation.cpp -o random_rotation
// To run use:
// > ./random_rotation

#pragma cling add_include_path("/usr/include/eigen3")
#define _USE_MATH_DEFINES
#include <cmath>
#include <ctime>

#include "Eigen/Dense"
#include <iostream>
#include <cstdlib>

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

int main(int argc, char* argv[]) {
    srand(time(0));
    std::cout << random_rotation() << "\n";
}
