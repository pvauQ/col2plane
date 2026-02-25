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
#include "lm_solver.h"



Col2Plane::Col2Plane(){
    //set marker world positions;
    this->marker_word_pos =  CctagFileHelper::TagWorldLocationFromFile(std::filesystem::path("tag_world_pos.txt"));    
    
    std::filesystem::path path("photodir/malli");
    std::filesystem::path model_path  =path ;
    colmap_transforms =  getAvgreprojError(model_path); // this gives transforms that have the error in there :)
    colmap_cam_params = getCameraParameters(model_path);

}

// cctag output previously saved to a file
void Col2Plane::withPrecalulated(std::string file){
    std::filesystem::path p = std::filesystem::current_path();
    img_tags = CctagFileHelper::readTagsFromtxt(p/file);
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

//get to see what tags are visible in how many images
// returns id and in how many images it is
std::map<int,int> Col2Plane::tagsVisibleInImages(){
    std::map<int,int> tag_in_images;
    //we trust that no duplicates exist
    for (const auto& tag : img_tags){
        for (const int id : tag.ids){
            tag_in_images[id]++;
        }
    }
    // decending sort
    std::vector<std::pair<int,int>> sorted_tags(tag_in_images.begin(), tag_in_images.end());
    std::sort(sorted_tags.rbegin(), sorted_tags.rend(), 
          [](const auto& a, const auto& b){ return a.second < b.second; });
    return tag_in_images;

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
        }
            //this filtering  uses reprojection error of pnp solve -> it's different than colmap reperror!!
            std::vector<matrixTransform> filtered_cams = FilterByError(tag_based_cam_trans,5.5);
            cameraPosOutput(filtered_cams);
    }break;






    case:: solve_mode::LM_SOLVE:{
        //TODO:  automaattinen valinta, ja tai arg parserilta valitaan tägit
        int n_cams_to_use = this->number_of_images_lm; //  how many cameras are used per marker 
        std::cout << "using " << n_cams_to_use << "cams in solver \n";
        int tags_to_use[3];
        int iter = 0;
        std::map<int,int> num_tags = tagsVisibleInImages();
        for (const auto& t : num_tags){
            assert(marker_word_pos.find(t.first) != marker_word_pos.end()); //täg found but not in tags_to_use.txt
            tags_to_use[iter] = t.first;
            iter++;
            if (iter >3) 
                break;
        }


        
        /*solveri tarvitsee cams,  ray_dirs, X_target ( world pos) 
        cams  =suoraan colmap kamerat ja niiden lokaatiot.
        ray_dirs = camera_rotation⁻1 *  normalize() K⁻1 * {u,v,1}^T )
        */
        // halutaan siis n kameraa jotka näkee markkerin x, ja tämä erikseen muutamalla markkerille.
        
        std::sort(colmap_transforms.begin(), colmap_transforms.end(), 
          [](const transform& a, const transform& b) {
              return a.error < b.error;
          });
        std::vector<tag_col_dir> image_infos ;
        //tägit kuvissa
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

        std::vector<tag_col_dir> use_for_solve = filterAndSortByTag(image_infos, tags_to_use[0]);
        std::vector<tag_col_dir> use_for_scale = filterAndSortByTag(image_infos, tags_to_use[1]);
        std::vector<tag_col_dir> use_for_3 = filterAndSortByTag(image_infos, tags_to_use[2]);

    //------------------------------------ solve part
    ///////////////////////////////////////////////////////////////////////////////////////////
        
        //todo refactor to use new solver nicely
    
        std::vector<Eigen::Vector3d> cams1 , cams2, cams3; 
        std::vector<Eigen::Vector3d> ray_dirs1, ray_dirs2, ray_dirs3;
        CollectCamRays(cams1,ray_dirs1,use_for_solve,tags_to_use[0],n_cams_to_use);
        CollectCamRays(cams2,ray_dirs2,use_for_scale,tags_to_use[1],n_cams_to_use);
        CollectCamRays(cams3,ray_dirs3,use_for_3,    tags_to_use[2],n_cams_to_use);

        //shity test for some coplanarity, if this is something that actually causes problems TODO:proper tests
        Eigen::Vector3d p0 = cams1[0], p1 = cams1[1], p2 = cams1[2], p3 = cams1[3];
        Eigen::Vector3d v1 = p1 - p0;
        Eigen::Vector3d v2 = p2 - p0;
        Eigen::Vector3d plane_normal = v1.cross(v2).normalized();
        Eigen::Vector3d test_vec = p3 - p0;
        double dotprod = test_vec.dot(plane_normal);
        std::cout << "cams1[0-3] planarity ( dot prod): " << dotprod  << "\n";

        
        // tag world pos
        std::vector<Eigen::Vector3d> world_positions;
        world_positions.push_back(marker_word_pos.find(tags_to_use[0])->second); // these fill the given vectors.
        world_positions.push_back(marker_word_pos.find(tags_to_use[1])->second);
        world_positions.push_back(marker_word_pos.find(tags_to_use[2])->second);
        std::vector<std::vector<Eigen::Vector3d>> cams;
        std::vector<std::vector<Eigen::Vector3d>> rays;
        cams.push_back(cams1);
        cams.push_back(cams2); 
        cams.push_back(cams3);
        rays.push_back(ray_dirs1);
        rays.push_back(ray_dirs2);
        rays.push_back(ray_dirs3);

        Eigen::VectorXd solution = lmDriver(cams, rays, world_positions); // SOLUTION 
        Eigen::Vector3d rot_angle_axis = solution.segment<3>(0);
        Eigen::Vector3d trans = solution.segment<3>(3);
        Eigen::VectorXd ray_depths = solution.segment(6, n_cams_to_use * 3 ); // first all rays for cam1(1,2,3) cam2 camd3

        //std::cout << " \n" << ray_depths.size()<< "\n";
        //std::cout << "rotation outside of solver"  << rot_angle_axis << "\n";
        Eigen::Matrix3d ROT = Eigen::AngleAxisd(rot_angle_axis.norm(), rot_angle_axis.normalized()).matrix();
        
        // establish scale part---------------------------------------------------------

        // we need to know distance between pair of markers in world frame and in s frame
        // given those scale is just ratio between those markers?? 
        double world_distance =(marker_word_pos.find(tags_to_use[0])->second - marker_word_pos.find(tags_to_use[1])->second).norm();
        // cam_c + ray_dir * ray_scale = target in s frame 
        Eigen::Vector3d tag_1_s = cams1[0] + ray_dirs1[0] *ray_depths[0];
        Eigen::Vector3d tag_2_s = cams2[0] + ray_dirs2[0] *ray_depths[n_cams_to_use];
        double s_frame_distance = (tag_1_s - tag_2_s).norm();
        double scale = world_distance / s_frame_distance;
        std::cout << "distance of pair " << tags_to_use[0] << " " << tags_to_use[1] << " in \n"\
                    << "s frame " << s_frame_distance << "\n" \
                    << "w frame " << world_distance << "\n" \
                    << "gives ratio " << scale << "\n";
        std::cout << "we got scale" << scale << "\n";

        Eigen::Quaterniond q_rot = Eigen::Quaterniond(ROT);
        transToFile(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(0.0, 0.0, 0.0), scale, "scale_trans.txt");
        transToFile(q_rot,trans, 1, "transform.txt");
     
        
    }break;
    default:
        std::cout << "not enough tags found \n";
        break;
    }
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


//calc one ray based on x,y  coordinates camera rotation.
//uses camera instrinsics and distortion from col2plane
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

// calc  ray directions based on cam tag pairs.
// builds results to vectors of 3 vectors. (cam locations, ray directions.)
void Col2Plane::CollectCamRays(std::vector<Eigen::Vector3d> & cams, 
                                std::vector<Eigen::Vector3d> & ray_dirs, 
                                std::vector<tag_col_dir> use_for_solve , 
                                int tag_to_use,
                                int n_cams_to_use){


    //std::cout << use_for_solve.size() <<  "  with images when  we need  "<<  n_cams_to_use << "cameras that see tag: " << tag_to_use << "\n";
    std::cout << "reprojection error ( colmap) for " << n_cams_to_use << ". cam " << use_for_solve[n_cams_to_use].camera_info.error <<"\n";
    assert(use_for_solve.size() > n_cams_to_use );
    
    for(int i = 0 ; i< n_cams_to_use;i++){
        
        Eigen::Vector3d c = (-use_for_solve[i].camera_info.rot.toRotationMatrix().transpose()) * use_for_solve[i].camera_info.translation;
        cams.push_back(c);
        for(int j = 0; j< use_for_solve[i].tag_info.ids.size();j++){
            if ( use_for_solve[i].tag_info.ids[j] == tag_to_use){
                double x = use_for_solve[i].tag_info.coordinates[j].first;
                double y = use_for_solve[i].tag_info.coordinates[j].second;
                //std::cout << "calculating  ray dir for "  << x << " "<< y << " " << use_for_solve[i].camera_info.filename << " to " << tag_to_use << "\n";
                ray_dirs.push_back( calculateCameraRay(x, y, use_for_solve[i].camera_info).normalized());
                //std::cout << "ray_dir dot products: " << ray_dirs[0].dot(ray_dirs.back()) << std::endl;    
                break;
            }
        }
    } 
 }

// filter so we have only tag_col_dirs that have tag defined by tag id
// results are sorted by colmap reproj error
 std::vector<tag_col_dir> Col2Plane::filterAndSortByTag(const std::vector<tag_col_dir>& image_infos, int tag_id) {
    std::vector<tag_col_dir> result;
    // ne joissa on täg = täg_id
    std::copy_if(image_infos.begin(), image_infos.end(), std::back_inserter(result), 
         [tag_id](const tag_col_dir& entry) {
            auto it = std::find(entry.tag_info.ids.begin(), entry.tag_info.ids.end(), tag_id);
            return it != entry.tag_info.ids.end();
         });
    // sort by name jotta voidaan delete unituqfilterAndSortByTag
    std::sort(result.begin(), result.end(),
          [](const tag_col_dir& a, const tag_col_dir& b) {
              return a.tag_info.image_name < b.tag_info.image_name;
          });
    result.erase(std::unique(result.begin(), result.end(),
          [](const tag_col_dir& a, const tag_col_dir& b) {
              return a.tag_info.image_name == b.tag_info.image_name;
          }), result.end());
    

    //lasty sort by error
    std::sort(result.begin(), result.end(),
        [](const tag_col_dir& a, const tag_col_dir& b) {
            return a.camera_info.error < b.camera_info.error;
        });

    std::cout << "we have " << result.size() << " possible imgs with tag " << tag_id << "\n";
    if (!result.empty()) {
        std::cout << " first " << result[0].camera_info.filename << "  with error: "  << result[0].camera_info.error << \
            " last " << result.back().camera_info.filename << "  with error: "  << result.back().camera_info.error << "\n";
    }

    return result;
}