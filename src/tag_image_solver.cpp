#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry>
#include "tag_image_solver.h"
#include <vector>
#include <iostream>
#include <assert.h>


// this should work for 4 points also?
//returns vector of transforms: these transforms represent camera location relative  to
// markers in the source image
std::vector<matrixTransform> solve3Tags1Img(std::vector<Eigen::Vector3d> &world_cords, std::vector<Eigen::Vector3d> &image_points,
                         Eigen::Matrix4d &intrinsics , Eigen::Vector4d &distorion)
{

    assert( image_points.size() == world_cords.size());
    Eigen::Matrix4d k_a = intrinsics;

    //// cv mailmaan hetkeksi
    //cv::Mat dist = (cv::Mat_<double>(1,4) << distorion[0], distorion[1], distorion[2], distorion[3]); 
    //cv::Mat K = (cv::Mat_<double>(3,3) << k_a(0,0), k_a(0,1), k_a(0,2),
    //                                        k_a(1,0), k_a(1,1), k_a(1,2),
    //                                        k_a(2,0), k_a(2,1), k_a(2,2));

    cv::Mat dist = (cv::Mat_<double>(1,4) << -0.1603, 0.1321, -0.0012, -0.0016); 
    cv::Mat K = (cv::Mat_<double>(3,3) << 4307.054, 0, 2592,
                                        0, 4293.814,  1728,
                                        0,      0,      1);
    
                                        

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
    std::vector<cv::Mat> rot_out, trans_out,rot_matices;

    if ( world_cords.size() == 4){
        std::cout <<" solving  with " << world_cords.size() << " points \n"
                << "im points: \n" << imagePoints
                << "\n world points:\n " <<  objectPoints <<"\n";

        solutions = cv::solveP3P(objectPoints, imagePoints,K, dist, rot_out, trans_out, cv::SOLVEPNP_P3P);    
    }
    std::cout << "Found "  <<solutions <<  " solutions ( rot matrix)" << std::endl;
    for (int i = 0; i < solutions; i++){
        cv::Mat r_matrix;
        cv::Rodrigues(rot_out[i],r_matrix);  
        rot_matices.push_back(r_matrix);        
        //std::cout<<r_matrix<< " \n";
    }
    assert(rot_matices.size() == solutions);

    // takaisin eigen maailmaan ->
    std::vector<Eigen::Matrix3d> rotations;
    std::vector<Eigen::Vector3d> translations;
    rotations.reserve(rot_matices.size());
    translations.reserve(trans_out.size());

    for(size_t i = 0; i < solutions; ++i) {
        assert(rot_matices[i].rows == 3 && rot_matices[i].cols == 3) ;
        assert(trans_out[i].rows == 3 && trans_out[i].cols == 1);
        rotations[i] = Eigen::Map<const Eigen::Matrix3d>(rot_matices[i].ptr<double>(), 3, 3);

        translations[i] = Eigen::Map<const Eigen::Vector3d>(trans_out[i].ptr<double>(), 3);

    }
    std::vector<matrixTransform> trans;
    for(size_t i = 0; i < solutions; ++i) {
        matrixTransform kissa;
        kissa.rotation = rotations[i];
        kissa.location = translations[i];
        trans.push_back(kissa);
    }
    return trans;
}
