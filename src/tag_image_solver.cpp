#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>  
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

    // cv mailmaan hetkeksi
    cv::Mat dist = (cv::Mat_<double>(1,4) << distorion[0], distorion[1], distorion[2], distorion[3]); 
    cv::Mat K = (cv::Mat_<double>(3,3) << k_a(0,0), k_a(0,1), k_a(0,2),
                                            k_a(1,0), k_a(1,1), k_a(1,2),
                                            k_a(2,0), k_a(2,1), k_a(2,2));
    
                             
    std::vector<cv::Point3d> objectPoints;
    std::vector<cv::Point2d> imagePoints;


    objectPoints.reserve(world_cords.size());
    imagePoints.reserve(image_points.size());

    for(size_t i = 0; i < world_cords.size(); ++i) {
        objectPoints.emplace_back(
            static_cast<double>(world_cords[i].x()),
            static_cast<double>(world_cords[i].y()), 
            static_cast<double>(world_cords[i].z()));
        imagePoints.emplace_back(
            static_cast<double>(image_points[i].x()),
            static_cast<double>(image_points[i].y()));
    }
    int solutions = 0;
    std::vector<cv::Mat> rot_out, trans_out,rot_matices;

    if ( world_cords.size() == 4){
        //std::cout <<" solving  with " << world_cords.size() << " points \n"
        //        << "im points: \n" << imagePoints
        //        << "\n world points:\n " <<  objectPoints <<"\n";


        //note: output of this transforms point in world frame to camera frame  P_c = R* p_w + t

        solutions = cv::solveP3P(objectPoints, imagePoints,K, dist, rot_out, trans_out, cv::SOLVEPNP_P3P);    
    }
    std::vector<double> reperr = calcReProjError(objectPoints, imagePoints,rot_out, trans_out, K,dist);

    //std::cout << "Found "  <<solutions <<  " solutions "  << std::endl;
    for (int i = 0; i < solutions; i++){
        //std::cout << "reprojection error  " << i << " " << reperr[i] << "\n";
        cv::Mat r_matrix;
        cv::Rodrigues(rot_out[i],r_matrix);  
        rot_matices.push_back(r_matrix);        
        
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
        //rotations[i] = Eigen::Map<const Eigen::Matrix3d>(rot_matices[i].ptr<double>(), 3, 3);

        cv::cv2eigen(rot_matices[i], rotations[i]); 
        
        translations[i] = Eigen::Map<const Eigen::Vector3d>(trans_out[i].ptr<double>(), 3);
    }
    //std::cout<<trans_out[0] <<"\n" << translations[0]<< " \n";

    std::vector<matrixTransform> trans;
    for(size_t i = 0; i < solutions; ++i) {
        matrixTransform kissa;
        kissa.error = reperr[i];
        kissa.rotation =  rotations[i].transpose(); // inverse of rotation world to cam 
        kissa.location =   -translations[i]; // inverse of translation world to cam
        trans.push_back(kissa);
    }
    return trans;
}








// n tag version, this returns only one solution.
matrixTransform solveNTags1Img(std::vector<Eigen::Vector3d> &world_cords, std::vector<Eigen::Vector3d> &image_points,
                         Eigen::Matrix4d &intrinsics , Eigen::Vector4d &distorion)
{

    assert( image_points.size() == world_cords.size());
    Eigen::Matrix4d k_a = intrinsics;

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

    bool  has_solution = false;
    cv::Mat rot_matrix;
    cv::Mat rot_out = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat trans_out = cv::Mat::zeros(3, 1, CV_64F);


    if ( world_cords.size() == 4){
        //note: output of this transforms point in world frame to camera frame  P_c = R* p_w + t
        has_solution = cv::solvePnP(objectPoints, imagePoints,K, dist, rot_out, trans_out, cv::SOLVEPNP_IPPE );    
    }
    if (has_solution == true){
        std::cout << "Found solution " <<  trans_out<<  std::endl;
        cv::Mat r_matrix;
        cv::Rodrigues(rot_out,r_matrix);  
        rot_matrix = r_matrix;      
    

        // takaisin eigen maailmaan ->
        Eigen::Matrix3d rotation;
        Eigen::Vector3d translation;
        cv::cv2eigen(rot_matrix, rotation);     // 3x3 Mat → Matrix3d
        cv::cv2eigen(trans_out, translation);   // 3x1 Mat → Vector3d
        matrixTransform trans;
    
        
        trans.rotation =  rotation.transpose(); // inverse of rotation world to cam 
        trans.location =  - translation; // inverse of translation world to cam
        return trans;
    }
    else{ //hack

        matrixTransform trans;
        trans.location = Eigen::Vector3d(0.0,0.0,0.0);
        return trans;
    }
}







std::vector<double> calcReProjError( std::vector<cv::Point3d> objectPoints, std::vector<cv::Point2d> imagePoints,
                        std::vector<cv::Mat> rvec, std::vector<cv::Mat> tvec,
                        cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    std::vector<double> repErrors;
    for(int i = 0; i< rvec.size() ; i++){
        std::vector<cv::Point2d> projected;
        cv::projectPoints(objectPoints, rvec[i], tvec[i], cameraMatrix, distCoeffs, projected);
        double error = cv::norm(imagePoints, projected, cv::NORM_L2);
        double reProjError = error / objectPoints.size();
        repErrors.push_back(reProjError);
        // halutaanko päätellä jotain yksittäisten pisteiden virheistä?
    }
    return repErrors;
}