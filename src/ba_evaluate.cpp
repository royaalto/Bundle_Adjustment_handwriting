#include "ba_eigen.h"
#include "utils.hpp"
#include <iostream>
#include <fstream>



int main(int argc, char** argv)
{
    std::string file_3dname = "p3d.txt";
    std::string file_2dname = "p2d.txt";
    int iteration = 10;
    std::vector<Eigen::Vector3d> vec_3d;
    std::vector<Eigen::Vector2d> vec_2d;
    bool read_suc = ReadTxt(file_2dname, file_3dname, vec_2d, vec_3d);
    if (!read_suc)
    {
        std::cout << "Read failed" << std::endl;
        return 0;
    }
    std::vector<Eigen::Matrix4d> vetex_poses;
    Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
    //get the camera pose
    vetex_poses.push_back(init_pose);
    //get the camera inertial parameter
    CamModel cam;
    cam.fx = 520.9;
    cam.fy = 521.0;
    cam.cx = 325.1;
    cam.cy = 249.7;

    BaEigen ba_eigen_test(vec_3d, vec_2d, vetex_poses, cam, iteration);
    ba_eigen_test.GaussNewton();
}
