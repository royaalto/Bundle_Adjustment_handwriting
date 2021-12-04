#ifndef BA_EIGEN_H
#define BA_EIGEN_H
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>

class BaEigen
{
public:
    BaEigen();
    ~BaEigen();

    Eigen::MatrixXd ComputeJacobian();

    double ComputeReprojectionError();

    Eigen::MatrixXd ComputeDeltax();

    void UpdatePose();

    void GuassNewton();
};

#endif
