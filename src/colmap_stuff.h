#ifndef colmapstuff
#define colmapstuff
#include <filesystem>
#include <cctag/CCTag.hpp>
#include "tag_image_solver.h"
#include <eigen3/Eigen/Geometry>
#include <vector>
struct cameraParams{
    Eigen::Matrix4d k; //intrincts
    Eigen::Vector4d distortion;
    cameraParams() = default;
    cameraParams(Eigen::Matrix4d k, Eigen::Vector4d d) : k(k), distortion(d) {};
};



std::vector<transform> getCameraTranforms(std::filesystem::path colmap_model_dir);
std::vector<transform> getAvgreprojError(std::filesystem::path colmap_model_dir);
cameraParams getCameraParameters(std::filesystem::path colmap_model_dir);




void transToFile(Eigen::Quaterniond rotation, Eigen::Vector3d trans, float scale, std::filesystem::path file);
void transMatrixToFile(Eigen::Matrix3d rot, Eigen::Vector3d trans , float scale, std::filesystem::path file, double info = 0.0 );
void camerasTofile(std::vector<transform> cameras);
void camerasTofile(std::vector<matrixTransform> cameras);

#endif
