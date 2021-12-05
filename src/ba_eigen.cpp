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
Eigen::MatrixXd BaEigen::ComputeJacobian(Eigen::Matrix4d update_pose)
{
    std::cout << "Start to compute the jacobian" << std::endl;
    //one jabian is 2*6
    // h = (2*6)^T * (2*6)
    Eigen::MatrixXd jaco = Eigen::MatrixXd::Zero(2 * point_3d_.size(), 6);
    Eigen::Matrix4d init_pose = update_pose;
    std::cout << "Init pose is " << std::endl << "Vertex poses : " << init_pose << std::endl;
    Eigen::MatrixXd jacobian_pose_i = Eigen::MatrixXd::Zero(2, 6);
    for (int i = 0; i < point_3d_.size(); i++)
    {
        Eigen::MatrixXd Rotation = init_pose.block<3, 3>(0, 0);
        Eigen::MatrixXd translation = init_pose.block<3, 1>(0, 3);
        Eigen::Vector3d update_point_pose = Rotation * point_3d_[i] + translation;
        double x = update_point_pose(0, 0);
        double y = update_point_pose(1, 0);
        double z = update_point_pose(2, 0);  //big mistake different bracket make mistakes
        double z_2 = z * z;
        if (true)
        {
            std::cout << "rotation " << Rotation << std::endl;
            std::cout << "translation" << translation << std::endl;
            std::cout << "update_point_pose" << update_point_pose << std::endl;
            std::cout << "point3d : " << point_3d_[i] << std::endl;
            std::cout << "x : " << x << " y : " << y << " z: " << z << std::endl
                      << -1. / z * cam_model_.fx << std::endl;
        }
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

        std::cout << "put into the " << 2 * i << "row, 0 column" << std::endl;
        std::cout << "jacobian : " << jacobian_pose_i << std::endl;
        jaco.block<2, 6>(2 * i, 0) = jacobian_pose_i;
    }
    std::cout << "jacobian_pose" << jacobian_pose_i << std::endl;
    return jaco;
}
Eigen::MatrixXd BaEigen::ComputeHessian()
{
    Eigen::MatrixXd h_copy = jacobian_.transpose() * jacobian_;
    return h_copy;
}

Eigen::VectorXd BaEigen::ComputeDeltaX()
{
    std::cout << "Start to compute the deltax" << std::endl;
    // 6*1
    Eigen::MatrixXd g = -jacobian_.transpose() * reprojection_error_vec_;
    std::cout << " g rows : " << g.rows() << "g cols: " << g.cols() << std::endl;
    Eigen::MatrixXd delta_x = hessian_.ldlt().solve(g);
    std::cout << "delta x rows : " << delta_x.rows() << " cols: " << delta_x.cols() << std::endl;
    return delta_x;
}

double BaEigen::ComputeReprojectionError(Eigen::Matrix4d update_pose)
{
    std::cout << "Start to compute the reprojection error" << std::endl;
    double reprojection_error = 0;
    reprojection_error_vec_ = Eigen::VectorXd(2 * point_3d_.size(), 1);
    Eigen::MatrixXd init_pose = update_pose;
    for (int i = 0; i < point_3d_.size(); i++)
    {
        Eigen::MatrixXd Rotation = init_pose.block<3, 3>(0, 0);
        Eigen::MatrixXd translation = init_pose.block<3, 1>(0, 3);
        Eigen::Vector3d update_point_pose = Rotation * point_3d_[i] + translation;
        double x = update_point_pose(0, 0);
        double y = update_point_pose(1, 0);
        double z = update_point_pose(2, 0);
        if (true)
        {
            std::cout << "x : " << x << " y: " << y << " z: " << z;
        }
        //observation
        double ob_u = cam_model_.cx + cam_model_.fx * x / z;
        double ob_v = cam_model_.cy + cam_model_.fy * y / z;
        //error
        double err_u = point_2d_[i](0, 0) - ob_u;
        double err_v = point_2d_[i](1, 0) - ob_v;
        if (true)
        {
            std::cout << "err_u : " << err_u << " err_v: " << err_v << std::endl;
        }
        // error[ u ]
        //      [ v ]
        reprojection_error_vec_(i * 2, 0) = err_u;
        reprojection_error_vec_(i * 2 + 1, 0) = err_v;
        reprojection_error += (err_u * err_u + err_v * err_v);
    }
    return reprojection_error / point_3d_.size();
}

Eigen::Matrix3d BaEigen::se2RotationMatrix(Eigen::Vector3d rot_vec)
{
    std::cout << "rodrigues2rot" << std::endl;
    // 初始化旋转矩阵
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    // 求旋转向量的反对称矩阵
    Eigen::Matrix3d skewW;
    skewW << 0.0, -rot_vec(2), rot_vec(1), rot_vec(2), 0.0, -rot_vec(0), -rot_vec(1), rot_vec(0), 0.0;

    // 求旋转向量的角度
    double omega_norm = rot_vec.norm();

    // 通过罗德里格斯公式把旋转向量转换成旋转矩阵
    if (omega_norm > std::numeric_limits<double>::epsilon())
    {
        R = R + std::sin(omega_norm) / omega_norm * skewW +
            (1 - std::cos(omega_norm)) / (omega_norm * omega_norm) * (skewW * skewW);
    }

    return R;
}

void BaEigen::UpdateSEPose(const Eigen::VectorXd& delta_x)
{
    //delta is se3 because we use ipsilon to calculate the ba
    //lie the second 3 vectors are rotation
    Eigen::Vector3d rot_vec = delta_x.block<3, 1>(3, 0);
    Eigen::Matrix3d rot_ = se2RotationMatrix(rot_vec);
    update_pose_.block<3, 3>(0, 0) = rot_;
    update_pose_.block<3, 1>(0, 3) = delta_x.block<3, 1>(0, 0);
    update_pose_(3, 3) = 1;
}
void BaEigen::GaussNewton()
{
    update_pose_ = vertex_poses_.at(0);
    for (unsigned int i = 0; i < iteration_; i++)
    {
        jacobian_ = ComputeJacobian(update_pose_);
        std::cout << "jacobian " << std::endl << jacobian_ << std::endl;
        hessian_ = ComputeHessian();
        double reprojection_error;
        reprojection_error = ComputeReprojectionError(update_pose_);
        std::cout << "reprojection error is : " << reprojection_error << std::endl;
        // H delta x = -J e; delta x -> SE4 4*4 se3 6*1
        Eigen::VectorXd delta_x = ComputeDeltaX();
        UpdateSEPose(delta_x);
    }
}
