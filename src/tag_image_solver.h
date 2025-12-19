#ifndef TAGSOLVER_H
#define TAGSOLVER_H
#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <string>

struct transform{
    int id;
    std::string filename;
    Eigen::Quaterniond rot;
    Eigen::Vector3d translation;
    transform(Eigen::Quaterniond q, Eigen::Vector3d t)  : rot(q), translation(t) {}
    transform(int id ,Eigen::Quaterniond q, Eigen::Vector3d t)  :id(id), rot(q), translation(t) {}
    transform(int id, std::string f,Eigen::Quaterniond q, Eigen::Vector3d t)  :id(id),filename(f), rot(q), translation(t) {}
};

struct matrixTransform{
    Eigen::Matrix3d rotation;
    Eigen::Vector3d location;
};


std::vector<matrixTransform> solve3Tags1Img(std::vector<Eigen::Vector3d> &world_cords, std::vector<Eigen::Vector3d> &image_points,
                         Eigen::Matrix4d &intrincts , Eigen::Vector4d &distorion);

    
transform diffrence(transform &a, transform &b );


#endif