#include "Math/InverseKinematics.h"
#include "Math/Jacobian.h"
#include <Eigen/QR>    


void nullspace_projector(const Eigen::Matrix<float, 7, 1> q,
                         Eigen::Matrix<float, 7, 7>& P)
{
    Eigen::Matrix<float, 6, 7> J;
    
    jacobian(q, J);

    Eigen::MatrixXf jinv = J.completeOrthogonalDecomposition().pseudoInverse();
    
    P = Eigen::Matrix<float, 7, 7>::Identity(7, 7) - jinv * J;
}

void inverse_kinematics(const Eigen::Matrix<float, 7, 1> q,
                        const Eigen::Matrix<float, 6, 1> xp,
                        Eigen::Matrix<float, 7, 1> &qp) {
    Eigen::Matrix<float, 6, 7> J;
    jacobian(q, J);

    Eigen::MatrixXf jinv = J.completeOrthogonalDecomposition().pseudoInverse();
    qp = (jinv * xp);

    Eigen::Matrix<float, 7, 7> P;
    nullspace_projector(q, P);
    Eigen::Matrix<float, 7, 1> z0 = Eigen::Matrix<float, 7, 1>::Zero();
    qp = qp + (P * z0);
    
    qp = qp RAD2DEG;
}
