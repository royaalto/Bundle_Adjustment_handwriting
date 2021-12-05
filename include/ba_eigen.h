#ifndef BA_EIGEN_H
#define BA_EIGEN_H
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>

class BaEigen
{
public:
    BaEigen(const std::vector<Eigen::Vector3d>& vec_3d, const std::vector<Eigen::Vector2d>& vec2d, int iteration);
    ~BaEigen();

    Eigen::MatrixXd ComputeJacobian();

    Eigen::MatrixXd ComputeHessian();

    double ComputeReprojectionError();

    Eigen::MatrixXd ComputeDeltax();

    void UpdatePose();

    void GaussNewton();

private:
    int iteration_;
    std::vector<Eigen::Vector2d> point_2d_;
    std::vector<Eigen::Vector3d> point_3d_;
    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd hessian_;
};

#endif
