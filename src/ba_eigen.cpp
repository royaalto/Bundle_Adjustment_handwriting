#include "ba_eigen.h"

BaEigen::BaEigen(const std::vector<Eigen::Vector3d>& vec_3d, const std::vector<Eigen::Vector2d>& vec_2d, int iteration)
    : point_2d_{vec_2d}
    , point_3d_{vec_3d}
    , iteration_{iteration}
{
}

BaEigen::~BaEigen()
{

}
Eigen::MatrixXd BaEigen::ComputeJacobian()
{

}
Eigen::MatrixXd BaEigen::ComputeHessian()
{

}

void BaEigen::GaussNewton()
{
    for (unsigned int i = 0; i < 10; i++)
    {
        jacobian_ = ComputeJacobian();
        hessian_ = ComputeHessian();
    }
}
