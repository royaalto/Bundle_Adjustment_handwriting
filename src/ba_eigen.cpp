#include "ba_eigen.h"
#include <iostream>

BaEigen::BaEigen(const std::vector<Eigen::Vector3d>& vec_3d,
                 const std::vector<Eigen::Vector2d>& vec_2d,
                 std::vector<Eigen::Matrix4d> vertex_poses,
                 CamModel cam,
                 int iteration)
    : point_2d_{vec_2d}
    , point_3d_{vec_3d}
    , iteration_{iteration}
    , vertex_poses_{vertex_poses}
    , cam_model_(cam)
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
    Eigen::Matrix4d init_pose = vertex_poses_.at(0);
    std::cout << "Init pose is " << std::endl << "Vertex poses : " << init_pose << std::endl;
    Eigen::MatrixXd jacobian_pose_i = Eigen::MatrixXd::Zero(2, 6);
    for (int i = 0; i < point_3d_.size(); i++)
    {
        Eigen::MatrixXd Rotation = init_pose.block<3, 3>(0, 0);
        Eigen::MatrixXd translation = init_pose.block<3, 1>(0, 3);
        Eigen::Vector3d update_point_pose = Rotation * point_3d_[i] + translation;
        double x = update_point_pose[0, 0];
        double y = update_point_pose[1, 0];
        double z = update_point_pose[2, 0];
        double z_2 = z * z;
        //Jacobian deltat delta f/delta t
        jacobian_pose_i(0, 0) = -1. / z * cam_model_.fx;
        jacobian_pose_i(0, 1) = 0;
        jacobian_pose_i(0, 2) = x / z_2 * cam_model_.fx;
        jacobian_pose_i(0, 3) = x * y / z_2 * cam_model_.fx;
        jacobian_pose_i(0, 4) = -(1 + (x * x / z_2)) * cam_model_.fx;
        jacobian_pose_i(0, 5) = y / z * cam_model_.fx;
        jacobian_pose_i(1, 0) = 0;
        jacobian_pose_i(1, 1) = -1. / z * cam_model_.fy;
        jacobian_pose_i(1, 2) = y / z_2 * cam_model_.fy;
        jacobian_pose_i(1, 3) = (1 + (y * y / z_2)) * cam_model_.fy;
        jacobian_pose_i(1, 4) = -x * y / z_2 * cam_model_.fy;
        jacobian_pose_i(1, 5) = -x / z * cam_model_.fy;

        std::cout<<"put into the "<< 2*i << "row, 0 column"<<std::endl;
        std::cout<<"jacobian : "<< jacobian_pose_i<<std::endl;
        jaco.block<2,6>(2* i, 0)= jacobian_pose_i;
    }
    std::cout<<"jacobian_pose"<<jacobian_pose_i<<std::endl;
    return jaco;
}
Eigen::MatrixXd BaEigen::ComputeHessian()
{
    Eigen::MatrixXd h_copy = jacobian_.transpose() * jacobian_;
    return h_copy;
}

double BaEigen::ComputeReprojectionError()
{
    std::cout<<"Start to compute the reprojection error" <<std::endl;
}

void BaEigen::GaussNewton()
{
    for (unsigned int i = 0; i < iteration_; i++)
    {
        jacobian_ = ComputeJacobian();
        std::cout<<"jacobian "<<std::endl <<jacobian_<<std::endl;
        hessian_ = ComputeHessian();

        ComputeReprojectionError();

    }
}
