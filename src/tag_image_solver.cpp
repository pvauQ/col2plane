#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry>
#include "tag_image_solver.h"
#include <vector>
#include <iostream>
#include <assert.h>


// this should work for 4 points also?
//returns vector of transforms: these transforms represent camera location relative  to
// markers in the source image
std::vector<transform> solve3Tags1Img(std::vector<Eigen::Vector3d> world_cords, std::vector<Eigen::Vector3d> image_points,
                         Eigen::Matrix4d intrincts , Eigen::Vector4d distorion)
{

    assert( image_points.size() == world_cords.size());
    Eigen::Matrix4d k_a = intrincts;

    // cv mailmaan hetkeksi
    cv::Mat dist = (cv::Mat_<double>(1,4) << distorion[0], distorion[1], distorion[2], distorion[3]); 
    cv::Mat K = (cv::Mat_<double>(3,3) << k_a(0,0), k_a(0,1), k_a(0,2),
                                            k_a(1,0), k_a(1,1), k_a(1,2),
                                            k_a(2,0), k_a(2,1), k_a(2,2));

    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;

    objectPoints.reserve(world_cords.size());
    imagePoints.reserve(image_points.size());

    for(size_t i = 0; i < world_cords.size(); ++i) {
        objectPoints.emplace_back(
            static_cast<float>(world_cords[i].x()),
            static_cast<float>(world_cords[i].y()), 
            static_cast<float>(world_cords[i].z()));
        imagePoints.emplace_back(
            static_cast<float>(image_points[i].x()),
            static_cast<float>(image_points[i].y()));
    }
    int solutions = 0;
    std::vector<cv::Mat> rot_out, trans_out;

    if ( world_cords.size() == 3){
        std::cout <<" solving  with 3 points \n"
                << "im points: " << imagePoints
                << "\n world points: " <<  objectPoints <<"\n";

        solutions = cv::solveP3P(objectPoints, imagePoints,K, dist, rot_out, trans_out, cv::SOLVEPNP_P3P);    
    }
    else if (world_cords.size() == 4){
        solutions = cv::solvePnP(objectPoints, imagePoints,K, dist, rot_out, trans_out);
    }
    std::cout << "Found "  <<solutions <<  " solutions" << std::endl;
    
    
    // pois cv maailmasta:
    std::vector<Eigen::Vector3d> rotations_rodri; // this can be deleted
    std::vector<Eigen::Quaterniond> rot_quat;
    std::vector<Eigen::Vector3d> translations;

    std::vector<transform> transforms;//quat+trans

    for(size_t i = 0; i < rot_out.size(); ++i) { //AI
        rotations_rodri.emplace_back(Eigen::Map<Eigen::Vector3d>(rot_out[i].ptr<double>()));
        translations.emplace_back(Eigen::Map<Eigen::Vector3d>(trans_out[i].ptr<double>()));
        
        Eigen::AngleAxisd angle_axis(rotations_rodri[i].norm(), rotations_rodri[i].normalized());
        Eigen::Quaterniond quat(angle_axis);
        rot_quat.push_back(quat);
        //sanity
        std::cout << translations[i] <<  quat << "\n" ;
        transforms.push_back(transform(rot_quat[0],translations[0]));
    }
    return transforms;
}