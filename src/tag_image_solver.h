#ifndef TAGSOLVER_H
#define TAGSOLVER_H
#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry>
#include <vector>

struct transform{
    Eigen::Quaterniond rot;
    Eigen::Vector3d translation;
    transform(Eigen::Quaterniond q, Eigen::Vector3d t)  : rot(q), translation(t) {}
};


std::vector<transform> solve3Tags1Img(std::vector<Eigen::Vector3d> world_cords, std::vector<Eigen::Vector3d> image_points,
                         Eigen::Matrix3d intrincts , Eigen::Vector4d distorion);

    



#endif