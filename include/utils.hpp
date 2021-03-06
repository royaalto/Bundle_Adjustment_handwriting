#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <fstream>

bool ReadTxt(const std::string& file_2d,
             const std::string& file_3d,
             std::vector<Eigen::Vector2d>& vector_p2d,
             std::vector<Eigen::Vector3d>& vector_p3d)
{
    std::ifstream fp3d(file_3d);
    if (!fp3d)
    {
        std::cout << "No " << file_3d << std::endl;
        return false;
    }
    else
    {
        while (!fp3d.eof())
        {
            double p0, p1, p2;
            fp3d >> p0;
            fp3d >> p1;
            fp3d >> p2;

            Eigen::Vector3d Point3d;
            Point3d << p0, p1, p2;
            vector_p3d.push_back(Point3d);
        }
    }
    std::ifstream fp2d(file_2d);
    if (!fp2d)
    {
        std::cout << "No " << file_2d << std::endl;
        return false;
    }
    else
    {
        while (!fp2d.eof())
        {
            double p0, p1;
            fp2d >> p0;
            fp2d >> p1;

            Eigen::Vector2d Point2d;
            Point2d<<p0, p1;
            vector_p2d.push_back(Point2d);
        }
    }
    std::cout<<"Read 3d Points : "<<vector_p3d.size()<< " Read 2d Points : " <<vector_p2d.size()<<std::endl;
    assert(vector_p2d.size() == vector_p3d.size());
    return true;
}
