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



Col2Plane::Col2Plane(){
    //set marker world positions;
    // asume wrong order as printed clocwise starting from upper left -> 3,1,2,0
    this->marker_word_pos.emplace(0,Eigen::Vector3d(0.0,  0.0,  0.0));
    this->marker_word_pos.emplace(3,Eigen::Vector3d(0.0,  0.121,  0.0));
    this->marker_word_pos.emplace(1,Eigen::Vector3d(0.195,  0.121,  0.05));
    this->marker_word_pos.emplace(2,Eigen::Vector3d(0.195,  0.0,  0.0));
    
}


// cctag output previously saved to a file
void Col2Plane::withPrecalulated(std::string file){
    std::filesystem::path p = std::filesystem::current_path();
    tags = CctagFileHelper::readTagsFromtxt(p/file);
    //this->printTagInfos();
}

void Col2Plane::calcCctag(){

    std::filesystem::path path("photodir/malli");
    std::filesystem::path model_path  =path ;
    tags = CctagFileHelper::readTagsFromImages(path);
    //this->printTagInfos();

}
void Col2Plane::printTagInfos(){
    for( auto& tag : tags){
        std::cout << tag.image_name << " with: " <<  tag.ids.size() << " tags \n";
        for(int i = 0; i< tag.coordinates.size(); i++){
            std::cout << tag.coordinates[i].first << " " << tag.coordinates[i].second << " id:" << tag.ids[i] <<"\n";
        }
        std::cout << std::endl;
    }

}


void Col2Plane::col2CctSpace(int num_tags){

    std::filesystem::path path("photodir/malli");
    std::filesystem::path model_path  =path ;
    // colmapin tuottamat kamerat ja näiden transformit
    std::vector<transform> colmap_transforms;    
    colmap_transforms =  getCameraTranforms(model_path);
    cameraParams colmap_cam_params(getCameraParameters(model_path));


    std::vector<CctagFileHelper::tagInfo>  ims_to_use;
    std::vector<matrixTransform> tag_based_cam_trans;
    //CctagFileHelper::tagInfo im;
    switch (num_tags)
    {
    case 4:{
            for(const auto& im: tags){
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
                    matrixTransform tmp = output[0];
                    tmp.filename = im.image_name; // opencv sorts these by error, so first one should be the "correct ..."
                    tmp.location = tmp.location;
                    tmp.derived_location = tmp.rotation * -tmp.location; //matrix * translation
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
    
    default:
        std::cout << "not enough tags found \n";
        break;
    }

    cameraPosOutput(tag_based_cam_trans);



    return;

}

// Writes file to be used in colmap model aligner
// This requires only raw locations of the cameras in the world <- from cctag -> p3p
void Col2Plane::cameraPosOutput(std::vector<matrixTransform>& transforms){
    if (transforms.size() < 3){
        std::cout << "least 3 cams needed for colmap model aligner we have " << transforms.size() << "\n";
        return;
    }
    //https://colmap.github.io/faq.html#geo-registration
    
    camerasTofile(transforms);


}

// variant for calculating and outputting the transform that takes colmap model to world coords..
void transformOutput(){};

