#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry>
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
    // asume wrong order as printed clocwise starting from upper left -> 3,1,2,0
    //this->marker_word_pos.emplace(0,Eigen::Vector3d(0.0,  0.0,  0.0));
    //this->marker_word_pos.emplace(3,Eigen::Vector3d(0.0,  0.121,  0.0));
    //this->marker_word_pos.emplace(1,Eigen::Vector3d(0.195,  0.121,  0.05));
    //this->marker_word_pos.emplace(2,Eigen::Vector3d(0.195,  0.0,  0.0));
    this->marker_word_pos =  CctagFileHelper::TagWorldLocationFromFile(std::filesystem::path("tag_world_pos.txt"));
    
}


// cctag output previously saved to a file
void Col2Plane::withPrecalulated(std::string file){
    std::filesystem::path p = std::filesystem::current_path();
    img_tags = CctagFileHelper::readTagsFromtxt(p/file);
    //this->printTagInfos();
}

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

    std::filesystem::path path("photodir/malli");
    std::filesystem::path model_path  =path ;
    // colmapin tuottamat kamerat ja näiden transformit
    std::vector<transform> colmap_transforms;    
    colmap_transforms =  getCameraTranforms(model_path);
    cameraParams colmap_cam_params(getCameraParameters(model_path));


    std::vector<CctagFileHelper::tagInfo>  ims_to_use;
    std::vector<matrixTransform> tag_based_cam_trans;
    //CctagFileHelper::tagInfo im;
    switch (mode)
    {
    case solve_mode::PP3_SOLVE:{
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
        
        /*functori tarvitsee cams,  ray_dirs, X_target ( world pos) 
        cams  =suoraan colmap kamerat ja niiden lokaatiot.
        ray_dirs = camera_rotation⁻1 *  normalize() K⁻1 * {u,v,1}^T )
        */
        // halutaan siis n kameraa jotka näkee markkerin x, ja tämä erikseen muutamalla markkerille.
        // simplehack for now just use 3 first cams that see tags
        
        struct tag_col_dir
        {
            CctagFileHelper::tagInfo tag_info;
            transform camera_info;
            std::vector<Eigen::Vector3d> ray_dirs;
        };

        std::vector<tag_col_dir> image_infos ;
        for(const auto& tagI: img_tags){
            tag_col_dir tmp; 
            tmp.tag_info =tagI;
            for(const auto& trans: colmap_transforms){
                if( tagI.image_name == trans.filename){
                    tmp.camera_info = trans;
                    break;
                }
            image_infos.push_back(tmp);
            }
        }
        //calc ray dirs
        for(auto& img : image_infos){
            for(int i = 0; i< img.tag_info.ids.size();i++){
                Eigen::Matrix3d ROT = img.camera_info.rot.toRotationMatrix();
                Eigen::Matrix3d K = colmap_cam_params.k.block<3,3>(0,0);; // we dont use distortions here at all. -> this might be a problem.
                if (K.determinant() == 0) {
                    std::cerr << "K = ain't invertible this is broken ..."; 
                }
                //care: is this correct
                Eigen::Vector3d img_cord(img.tag_info.coordinates[i].first, img.tag_info.coordinates[i].second, 0.0 );
                Eigen::Vector3d ray_dir = ROT.transpose()  * (K.inverse() * img_cord).normalized();
                img.ray_dirs.push_back(ray_dir);// care. id can be read ffrom tag_info_ids
            }
        }


        // just a hack for a while.
        //select tag to solve for, use 3 images for now
        int tag_to_use = 1;
        std::vector<tag_col_dir> use_for_solve;
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
        std::cout << "before using lm solve we have "  << use_for_solve.size()  << " possible imgs with tag " << tag_to_use << "\n";




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
void transformOutput(){};

