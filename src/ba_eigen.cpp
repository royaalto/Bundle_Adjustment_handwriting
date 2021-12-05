#include "ba_eigen.h"
#include <iostream>

BaEigen::BaEigen(const std::vector<Eigen::Vector3d>& vec_3d,
                 const std::vector<Eigen::Vector2d>& vec_2d,
                 std::vector<Eigen::Matrix4d> vertex_poses,
                 int iteration)
    : point_2d_{vec_2d}
    , point_3d_{vec_3d}
    , iteration_{iteration}
    , vertex_poses_{vertex_poses_}
{
}

BaEigen::~BaEigen()
{
}
Eigen::MatrixXd BaEigen::ComputeJacobian()
{
    std::cout << "Start to compute the jacobian" << std::endl;
    //one jabian is 2*6
    // h = (2*6)^T *
    Eigen::MatrixXd jaco = Eigen::MatrixXd::Zero(2 * point_3d_.size(), 6);
    for (int i = 0; i < point_3d_.size(); i++)
    {
    }
}
Eigen::MatrixXd BaEigen::ComputeHessian()
{
}

void BaEigen::GaussNewton()
{
    for (unsigned int i = 0; i < iteration_; i++)
    {
        jacobian_ = ComputeJacobian();
        hessian_ = ComputeHessian();
    }
}
