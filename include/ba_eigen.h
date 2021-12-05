#ifndef BA_EIGEN_H
#define BA_EIGEN_H
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
struct CamModel
{
    double fx;
    double fy;
    double cx;
    double cy;

};

class BaEigen
{
public:
    BaEigen(const std::vector<Eigen::Vector3d>& vec_3d, const std::vector<Eigen::Vector2d>& vec2d, std::vector<Eigen::Matrix4d> vertex_poses, CamModel cam, int iteration);
    ~BaEigen();

    Eigen::MatrixXd ComputeJacobian(Eigen::Matrix4d update_pose);

    Eigen::MatrixXd ComputeHessian();

    double ComputeReprojectionError(Eigen::Matrix4d update_pose);

    Eigen::VectorXd ComputeDeltaX();

    void UpdatePose();

    void GaussNewton();

    void UpdateSEPose(const Eigen::VectorXd& delta_x);

    Eigen::Matrix3d se2RotationMatrix(Eigen::Vector3d rot_vec);

private:
    int iteration_;
    std::vector<Eigen::Vector2d> point_2d_;
    std::vector<Eigen::Vector3d> point_3d_;
    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd hessian_;
    std::vector<Eigen::Matrix4d> vertex_poses_;
    CamModel cam_model_;
    Eigen::VectorXd reprojection_error_vec_;
    Eigen::Matrix4d update_pose_;
};

#endif
