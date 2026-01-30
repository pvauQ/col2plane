#include <iostream>
#include <eigen3/Eigen/Geometry>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <filesystem>
#include <iostream>
#include <algorithm>
#include "col2plane.h"
#include "tag_image_solver.h"
#include "tag_locations.h"
#include "colmap_stuff.h"
#include "tags_to_file.h"
#include "lm_sover.h"



Col2Plane::Col2Plane(){
    //set marker world positions;
    this->marker_word_pos =  CctagFileHelper::TagWorldLocationFromFile(std::filesystem::path("tag_world_pos.txt"));    
    
    std::filesystem::path path("photodir/malli");
    std::filesystem::path model_path  =path ;
    // colmapin tuottamat kamerat ja näiden transformit    
    //colmap_transforms =  getCameraTranforms(model_path);
    colmap_transforms =  getAvgreprojError(model_path); // this gives transforms that have the error in there :)
    colmap_cam_params = getCameraParameters(model_path);

}


// cctag output previously saved to a file
void Col2Plane::withPrecalulated(std::string file){
    std::filesystem::path p = std::filesystem::current_path();
    img_tags = CctagFileHelper::readTagsFromtxt(p/file);
    //this->printTagInfos();
}

//use cctag to find markers
void Col2Plane::calcCctag(){

    std::filesystem::path path("photodir/malli");
    std::filesystem::path model_path  =path ;
    img_tags = CctagFileHelper::readTagsFromImages(path);
    //this->printTagInfos();

}
void Col2Plane::printTagInfos(){
    for( auto& tag : img_tags){
        std::cout << tag.image_name << " with: " <<  tag.ids.size() << " tags \n";
        for(int i = 0; i< tag.coordinates.size(); i++){
            std::cout << tag.coordinates[i].first << " " << tag.coordinates[i].second << " id:" << tag.ids[i] <<"\n";
        }
        std::cout << std::endl;
    }

}


void Col2Plane::col2CctSpace(solve_mode mode){

    std::vector<CctagFileHelper::tagInfo>  ims_to_use;
    std::vector<matrixTransform> tag_based_cam_trans;
    //CctagFileHelper::tagInfo im;
    switch (mode)
    {
    case solve_mode::P3P_SOLVE:{
        for(const auto& im: img_tags){
            if (im.ids.size() == 4)
            ims_to_use.push_back(im);
        }
        std::cout << ims_to_use.size() << " images with 4 tags \n";      
        for (const auto& im : ims_to_use){

            std::vector<Eigen::Vector3d> world_pos;
            std::vector<Eigen::Vector3d> image_pos;
            for(int i = 0 ; i< im.ids.size(); i++){
                int id = im.ids[i];
                world_pos.push_back(marker_word_pos.find(id)->second);
                Eigen::Vector3d im_pos = Eigen::Vector3d(im.coordinates[i].first, im.coordinates[i].second, 0.0);
                image_pos.push_back(im_pos);;
                // note -> onko koordinaatit nyt oikeassa järjestyksess' ( x , y ,z )
            }
            std::vector<matrixTransform> output =  solve3Tags1Img(world_pos, image_pos,colmap_cam_params.k, colmap_cam_params.distortion);
            if (output.size() >0){
                matrixTransform tmp = output[0]; // first one has alway the lowest error
                tmp.filename = im.image_name; 
                tmp.location = tmp.location;
                tmp.derived_location = tmp.rotation * tmp.location; //matrix * translation
                tag_based_cam_trans.push_back(tmp);
            }
            /*
            matrixTransform output =  solveNTags1Img(world_pos, image_pos,colmap_cam_params.k, colmap_cam_params.distortion);
            if(output.location != Eigen::Vector3d(0.0,0.0,0.0)){
                output.filename = im.image_name;
                output.derived_location = output.rotation * output.location; //mat*vec
                std::cout << output.derived_location << "\n";
                tag_based_cam_trans.push_back(output);
            }*/
        }
    }break;


    case:: solve_mode::LM_SOLVE:{

        int tag_to_use = 0; // lm solve using this
        int tag_to_scale =1; // scale using this after we have solution
        int n_cams_to_use = 5; //  how many cameras used; these must see the marker

        
        /*functori tarvitsee cams,  ray_dirs, X_target ( world pos) 
        cams  =suoraan colmap kamerat ja niiden lokaatiot.
        ray_dirs = camera_rotation⁻1 *  normalize() K⁻1 * {u,v,1}^T )
        */
        // halutaan siis n kameraa jotka näkee markkerin x, ja tämä erikseen muutamalla markkerille.
        
        std::sort(colmap_transforms.begin(), colmap_transforms.end(), 
          [](const transform& a, const transform& b) {
              return a.error < b.error;
          });

        std::vector<tag_col_dir> image_infos ;
        for(const auto& tagI: img_tags){
            tag_col_dir tmp; 
            tmp.tag_info =tagI;
            for(const auto& trans: colmap_transforms){
                if( tagI.image_name == trans.filename){
                    tmp.camera_info = trans;
                    image_infos.push_back(tmp);
                    break;
                }
            }
        }

        // let's make two vectors, for solving and for scaling

        std::vector<tag_col_dir> use_for_solve;
        std::vector<tag_col_dir> use_for_scale;
        std::copy_if(image_infos.begin(), image_infos.end(), std::back_inserter(use_for_solve), 
             [tag_to_use](const tag_col_dir& entry){
                auto it = std::find(entry.tag_info.ids.begin(), entry.tag_info.ids.end(), tag_to_use);
                if( it !=  entry.tag_info.ids.end()){
                    return true;
                }else{
                    return false;
                }
             });
        
        std::sort(use_for_solve.begin(), use_for_solve.end(),
              [](const tag_col_dir& a, const tag_col_dir& b) {
                  return a.tag_info.image_name < b.tag_info.image_name;
              });
        use_for_solve.erase(std::unique(use_for_solve.begin(), use_for_solve.end(),
              [](const tag_col_dir& a, const tag_col_dir& b) {
                  return a.tag_info.image_name == b.tag_info.image_name;
              }), use_for_solve.end());
        std::cout << "we have "  << use_for_solve.size()  << " possible imgs with tag " << tag_to_use << "\n";
        std::cout <<"first" << image_infos[0].camera_info.filename << "\n";

        //scale---------------------------------
        std::copy_if(image_infos.begin(), image_infos.end(), std::back_inserter(use_for_scale), 
        [tag_to_scale](const tag_col_dir& entry){
        auto it = std::find(entry.tag_info.ids.begin(), entry.tag_info.ids.end(), tag_to_scale);
        if( it !=  entry.tag_info.ids.end()){
            return true;
        }else{
            return false;
        }
        });

        std::sort(use_for_scale.begin(), use_for_scale.end(),
              [](const tag_col_dir& a, const tag_col_dir& b) {
                  return a.tag_info.image_name < b.tag_info.image_name;
              });
        use_for_scale.erase(std::unique(use_for_scale.begin(), use_for_scale.end(),
              [](const tag_col_dir& a, const tag_col_dir& b) {
                  return a.tag_info.image_name == b.tag_info.image_name;
              }), use_for_scale.end());
        std::cout << "we have "  << use_for_scale.size()  << " possible imgs with tag " << tag_to_scale << "\n";
        std::cout <<"first" << image_infos[0].camera_info.filename << "\n";

    //------------------------------------ solve part
    ///////////////////////////////////////////////

    /////////////////////////////////////////////
        



        std::vector<Eigen::Vector3d> cams1 , cams2; 
        std::vector<Eigen::Vector3d> ray_dirs1, ray_dirs2;
        CollectCamRays(cams1,ray_dirs1,use_for_solve,tag_to_use,n_cams_to_use);
        CollectCamRays(cams2,ray_dirs2,use_for_scale,tag_to_scale,n_cams_to_use);

        std::vector<Eigen::Vector3d> world_positions;
        world_positions.push_back(marker_word_pos.find(tag_to_use)->second); // these fill the given vectors.
        world_positions.push_back(marker_word_pos.find(tag_to_scale)->second);
        
        Eigen::VectorXd solution = lmDriver(cams1,ray_dirs1, cams2,ray_dirs2,  world_positions); // SOLUTION 
        Eigen::Vector3d rot_angle_axis = solution.segment<3>(0);
        Eigen::Vector3d trans = solution.segment<3>(3);
        Eigen::Matrix3d ROT = Eigen::AngleAxisd(rot_angle_axis.norm(), rot_angle_axis.normalized()).matrix();

        std::cout << " solver gave rotation (euler angles): \n " << ROT.eulerAngles(0, 1, 2) *  180.0/M_PI  << " DEG \n";

        //transToFile (Eigen::Quaterniond rotation, Eigen::Vector3d trans , float scale)
        
        Eigen::Quaterniond q_rot = Eigen::Quaterniond(ROT);
        transToFile(q_rot,trans, 1);




        /*
        // estabilish scale part

        // 1. Undistort imagepoints
        // 2. camera lokaatiot esitettävä oikein
        // 3.triangulate ( )
        
        ///triangulate use 3 cams. //TODO:MAKE loop nicer..
        assert(use_for_scale.size() >= 3);
        std::vector<Eigen::Matrix<double, 3, 4>> projections;
        std::vector<Eigen::Vector2d> image_points;

        for(int i  = 0; i<3; i++){
            Eigen::Matrix<double, 3, 4> tmp;


            tmp.block<3,3>(0,0) = use_for_scale[i].camera_info.rot.toRotationMatrix(); // should this be inverse no?
            tmp.col(3) = use_for_scale[i].camera_info.translation;
            tmp = colmap_cam_params.k.block<3,3>(0,0) * tmp;
            projections.push_back(tmp);

            //get tag coordinates
            for(int j =0 ; j< use_for_scale[i].tag_info.ids.size(); j++ ){
                if(use_for_scale[i].tag_info.ids[j] == tag_to_scale){
                    int id  = use_for_scale[i].tag_info.ids[j];
                    image_points.emplace_back(use_for_scale[i].tag_info.coordinates[j].first, use_for_scale[i].tag_info.coordinates[j].second);
                    break;
                }
            }
        }
        // undistort  points that we are going to use,
        std::vector<cv::Mat> undistorted_im_points;
        for(auto& ip : image_points){
            cv::Mat k_mat(3, 3, CV_64F);           // 3x3 double for K matrix
            cv::Mat distortion_mat(1, 5, CV_64F);  // Row vector for 5 distortion coeffs (common for COLMAP/FULL_OPENCV)
            Eigen::Matrix3d k_block = colmap_cam_params.k.block<3,3>(0,0);
            cv::eigen2cv(k_block, k_mat);
            cv::eigen2cv(colmap_cam_params.distortion, distortion_mat);
            // we use  only one point, but function expecpts vec/multiple points;
            std::vector<cv::Point2d> src_points = {cv::Point2d(ip.x(), ip.y())};
            std::vector<cv::Point2d> undistorted_points;

            cv::undistortPoints(src_points, undistorted_points, k_mat, distortion_mat);
            cv::perspectiveTransform(undistorted_points, undistorted_points, k_mat); // what the fuck
            cv::Mat undistorted(2,1, CV_64F);
            undistorted.at<double>(0,0) = undistorted_points[0].x;
            undistorted.at<double>(1,0) = undistorted_points[0].y;
            
            //cv::Mat undistorted(undistorted_points[0].x, undistorted_points[0].y;
            undistorted_im_points.push_back(undistorted);
        }


        cv::Mat P1, P2, P3, x1, x2, x3;
        Eigen::Matrix<double, 3, 4> proj0 = projections[0];
        Eigen::Matrix<double, 3, 4> proj1 = projections[1];
        Eigen::Matrix<double, 3, 4> proj2 = projections[2];
        cv::eigen2cv(proj0, P1);
        cv::eigen2cv(proj1, P2);
        cv::eigen2cv(proj2, P3);

        cv::Mat out4d1,out4d2,out4d3 ;
        cv::triangulatePoints(P1,P2,undistorted_im_points[0],undistorted_im_points[1],out4d1); 
        cv::triangulatePoints(P1,P3,undistorted_im_points[0],undistorted_im_points[2],out4d2);
        cv::triangulatePoints(P2,P3,undistorted_im_points[1],undistorted_im_points[2],out4d3);
        // output from triangulate is in homogenious cordinates

        // jump back to eigen
        Eigen::Vector4d out4d1_eigen, out4d2_eigen, out4d3_eigen;
        cv::cv2eigen(out4d1, out4d1_eigen);
        cv::cv2eigen(out4d2, out4d2_eigen);
        cv::cv2eigen(out4d3, out4d3_eigen);
        // back to cartesian coordinates
        Eigen::Vector3d out_3d1 = out4d1_eigen.head<3>() / out4d1_eigen[3];
        Eigen::Vector3d out_3d2 = out4d2_eigen.head<3>() / out4d2_eigen[3]; 
        Eigen::Vector3d out_3d3 = out4d3_eigen.head<3>() / out4d3_eigen[3];
        Eigen::Vector3d loc_s_space = (out_3d1 + out_3d2 + out_3d3) / 3.0; // avg

        // care, some nice metrics to do some fuckkery
        std::cout << " via triangulation we got location for tag " << tag_to_scale << " in s space\n" << loc_s_space << "\n" ;
        
        // skaalan selvitys
        float dist_in_world_frame =  (marker_word_pos.find(tag_to_use)->second  - marker_word_pos.find(tag_to_scale)->second).norm();
        
        //std::cout << " trans in here"<< trans << "\n";
        Eigen::Vector3d point0 = ROT.transpose() * (marker_word_pos.find(tag_to_use)->second - trans);
        float dist_in_s_frame = (point0 - loc_s_space).norm();
        std::cout << " tag " << tag_to_use << "  in s space\n" << point0 << "\n" ;
        
        std::cout << "distance of tag pair  in world " << tag_to_use << " " << tag_to_scale << " is " << dist_in_world_frame <<"\n";
        std::cout << "distance of tag pair in S " << tag_to_use << " " << tag_to_scale << " is " <<  dist_in_s_frame <<"\n";

        float scene_scale = dist_in_world_frame / dist_in_s_frame;
        std::cout << "scale modifier should be " << scene_scale << "\n";

        //transToFile (Eigen::Quaterniond rotation, Eigen::Vector3d trans , float scale)
        Eigen::Quaterniond q_rot = Eigen::Quaterniond(ROT);
        transToFile(q_rot,trans, scene_scale);
        */
    }break;
    default:
        std::cout << "not enough tags found \n";
        break;
    }

    
    std::vector<matrixTransform> filtered_cams = FilterByError(tag_based_cam_trans,1.5);
    cameraPosOutput(filtered_cams);
    return;

}




std::vector<matrixTransform> FilterByError(std::vector<matrixTransform> &  i_transforms , double max_error){
    std::vector<matrixTransform> out;
    //todo: laske avg virhe..
    for (const auto& trans : i_transforms){
        if ( trans.error < max_error)
        out.push_back(trans);
    }
    return out;
}
std::vector<matrixTransform> FilterBestN(std::vector<matrixTransform>   i_transforms , size_t num_out){
    std::vector<matrixTransform> out;
    // by value koska pidetään alkuperäinen alkuperäisenä..
     sort(i_transforms.begin(), i_transforms.end(),
             [](const matrixTransform a,const matrixTransform b ) {
        return a.error < b.error; 
    });
    size_t max_iters  = std::min(num_out, i_transforms.size());
    for (int i = 0; i <max_iters; i++){
        out.push_back(i_transforms[i]);
    }
    return out;
}


// Writes file to be used in colmap model aligner
// This requires only raw locations of the cameras in the world <- from cctag -> p3p
void Col2Plane::cameraPosOutput(std::vector<matrixTransform>& transforms){
    if (transforms.size() < 3){
        std::cerr << " output error least 3 cams needed for colmap model aligner we have " << transforms.size() << "\n";
        return;
    }
    std::cout << "writing " << transforms.size() << " cam locations" << "\n";
    //https://colmap.github.io/faq.html#geo-registration
    
    camerasTofile(transforms);

}

// variant for calculating and outputting the transform that takes colmap model to world coords..
//void Col2Plane::transformOutput(){};



Eigen::Vector3d Col2Plane::calculateCameraRay(double x,  double y, transform camera){
    
        // this is  to undistort via opencv
    cv::Mat k_mat(3, 3, CV_64F);           // 3x3 double for K matrix
    cv::Mat distortion_mat(1, 5, CV_64F);  // Row vector for 5 distortion coeffs (common for COLMAP/FULL_OPENCV)
    Eigen::Matrix3d k_block = colmap_cam_params.k.block<3,3>(0,0);
    cv::eigen2cv(k_block, k_mat);
    cv::eigen2cv(colmap_cam_params.distortion, distortion_mat);
    // we use  only one point, but function expecpts vec/multiple points;
    std::vector<cv::Point2d> src_points = {cv::Point2d(x, y)};
    std::vector<cv::Point2d> undistorted_points;

    cv::undistortPoints(src_points, undistorted_points, k_mat, distortion_mat);
    cv::Point2d undistorted = undistorted_points[0];

    Eigen::Vector3d tmp(undistorted.x,undistorted.y,1);
    Eigen::Matrix3d CAM_ROT =  camera.rot.toRotationMatrix();
    tmp = CAM_ROT.transpose() * tmp; // miten tämän pitäisi :: olla otin miinuksen pois edestä oli  ->-CAM_ROT.transpose() * tmp;
    return tmp.normalized();
    
}
// give in empty vectors will build into those.
void Col2Plane::CollectCamRays(std::vector<Eigen::Vector3d> & cams, 
                                std::vector<Eigen::Vector3d> & ray_dirs, 
                                std::vector<tag_col_dir> use_for_solve , 
                                int tag_to_use,
                                int n_cams_to_use){

    //functori tarvitsee cams,  ray_dirs, X_target ( world pos) 

    std::cout << use_for_solve.size() <<  "  with images when  we need  "<<  n_cams_to_use << "cameras that see tag: " << tag_to_use << "\n";
    std::cout << "error for " << n_cams_to_use << ". element " << use_for_solve[n_cams_to_use].camera_info.error <<"\n";
    assert(use_for_solve.size() > n_cams_to_use );
    
    for(int i = 0 ; i< n_cams_to_use;i++){
        
        //NEED TO LEARN ABOUT CAMERA STUFF SOME MORE, 
        // this 
        Eigen::Vector3d c = (-use_for_solve[i].camera_info.rot.toRotationMatrix().transpose()) * use_for_solve[i].camera_info.translation;
        
        //cams.push_back(use_for_solve[i].camera_info.translation);
        cams.push_back(c);
        //purkka
        for(int j = 0; j< use_for_solve[i].tag_info.ids.size();j++){
            if ( use_for_solve[i].tag_info.ids[j] == tag_to_use){
                double x = use_for_solve[i].tag_info.coordinates[j].first;
                double y = use_for_solve[i].tag_info.coordinates[j].second;
                //std::cout << "calculating  ray dir for "  << x << " "<< y << " " << use_for_solve[i].camera_info.filename << " to " << tag_to_use << "\n";
                ray_dirs.push_back( calculateCameraRay(x, y, use_for_solve[i].camera_info));
                //std::cout << "ray_dir dot products: " << ray_dirs[0].dot(ray_dirs.back()) << std::endl;    
                break;
            }
        }
    } 
 }