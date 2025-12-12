#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry>
#include <filesystem>
#include <iostream>
#include "src/three_p_solve.h"
#include "tag_image_solver.h"
#include "tag_locations.h"


int main(){

    std::filesystem::path path("photodir/malli");
    readTagsFromImages(path);
    // this just for testing
    std::vector<Eigen::Vector3d> world_points;
    std::vector<Eigen::Vector3d> imagePoints;
    world_points.emplace_back(0.0f,  0.0f,  0.0f);  
    world_points.emplace_back(0.105f, 0.0f,  0.0f); 
    world_points.emplace_back(0.0f,  0.184f, 0.0f); 

    imagePoints.emplace_back(1745.60f, 2047.55f, 0.0f);
    imagePoints.emplace_back(2141.38f, 2064.50f, 0.0f);
    imagePoints.emplace_back(1465.50f, 2519.01f, 0.0f);

    Eigen::Matrix3d intrincts;
                    intrincts << 4307.054, 0, 2592,
                            0, 4293.814,  1728,
                            0,      0,      1;
    Eigen::Vector4d distorion(-0.1603, 0.1321, -0.0012, -0.0016);
    solve3Tags1Img(world_points,imagePoints,intrincts,distorion);


    

}

