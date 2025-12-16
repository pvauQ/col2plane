#ifndef colmapstuff
#define colmapstuff
#include <filesystem>
#include <cctag/CCTag.hpp>
#include "tag_image_solver.h"
#include <eigen3/Eigen/Geometry>

struct cameraParams{
    Eigen::Matrix4d k; //intrincts
    Eigen::Vector4d distortion;
    cameraParams(Eigen::Matrix4d k, Eigen::Vector4d d) : k(k), distortion(d) {};
};



std::vector<transform> getCameraTranforms(std::filesystem::path colmap_model_dir);
cameraParams getCameraParameters(std::filesystem::path colmap_model_dir);

#endif
