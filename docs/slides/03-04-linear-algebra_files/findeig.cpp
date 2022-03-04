#include <iostream>

#include "Eigen/Eigenvalues"
#include "Eigen/Dense"

int main(int argc, char* argv[]) {
    Eigen::Matrix3d A;
    A << 1, 2, 3,
        4, 5, 6,
        2, 4, 6;

    Eigen::EigenSolver<Eigen::Matrix3d> solver(A);
    Eigen::VectorXcd Lambda = solver.eigenvalues();
    Eigen::MatrixXcd S = solver.eigenvectors();
    std::cout << "\n\nEigen values: \n";
    std::cout <<  Lambda << "\n";

    std::cout << "\n\nEigen vectors: \n";
    std::cout << S << "\n";

    Eigen::VectorXd real_Lambda = Lambda.real();
    std::cout << "\n\nEigen values (real part): \n";
    std::cout <<  real_Lambda << "\n";

    std::cout << "\n\nEigen vectors: \n";
    std::cout << S.real() << "\n";

    std::cout << "\n\nMin eigen value: \n";
    Eigen::Index min_eigvalue_index;
    real_Lambda.array().abs().minCoeff(&min_eigvalue_index);
    std::cout << real_Lambda(min_eigvalue_index) << "\n";

    std::cout << "\n\nEigen vector corresponding to min eigen value: \n";
    std::cout << S.col(min_eigvalue_index).real() << "\n";
}
