#ifndef TAGSOLVER_H
#define TAGSOLVER_H
#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <string>

struct transform{
    int id; // cam id
    std::string filename; // when used for camera
    Eigen::Quaterniond rot;
    Eigen::Vector3d translation;
    double error;
    std::map<int, Eigen::Vector2d> point_2d_coords; // id of point and it's 2d location in this.
    transform()=default;
    transform(Eigen::Quaterniond q, Eigen::Vector3d t)  : rot(q), translation(t) {}
    transform(int id ,Eigen::Quaterniond q, Eigen::Vector3d t)  :id(id), rot(q), translation(t) {}
    transform(int id, std::string f,Eigen::Quaterniond q, Eigen::Vector3d t)  :id(id),filename(f), rot(q), translation(t) {}
};

struct matrixTransform{
    Eigen::Vector3d derived_location;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d location;
    std::string filename;
    double error; // i use this to store reprojection error when transform represents camera location(calculate from cctags)
    matrixTransform() = default; 
    matrixTransform(Eigen::Vector3d vec) : location(vec){}
    matrixTransform(Eigen::Vector3d vec, Eigen::Matrix3d mat) : location(vec), rotation(mat) {}
};


std::vector<matrixTransform> solve3Tags1Img(std::vector<Eigen::Vector3d> &world_cords, std::vector<Eigen::Vector3d> &image_points,
                         Eigen::Matrix4d &intrincts , Eigen::Vector4d &distorion);

matrixTransform solveNTags1Img(std::vector<Eigen::Vector3d> &world_cords, std::vector<Eigen::Vector3d> &image_points,
                         Eigen::Matrix4d &intrincts , Eigen::Vector4d &distorion);

std::vector<double> calcReProjError( std::vector<cv::Point3d> objectPoints, std::vector<cv::Point2d> imagePoints,
                        std::vector<cv::Mat> rvec, std::vector<cv::Mat> tvec,
                        cv::Mat cameraMatrix, cv::Mat distCoeffs);


#endif