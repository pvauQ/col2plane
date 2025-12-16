#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry>
#include <filesystem>
#include <iostream>
#include "src/three_p_solve.h"
#include "tag_image_solver.h"
#include "tag_locations.h"
#include "colmap_stuff.h"

int main(){

    
    std::filesystem::path path("photodir/malli");
    std::filesystem::path model_path  =path / "model";
    //colmap osuus
    // colmapin tuottamat kamerat ja näiden transformit
    std::vector<transform> colmap_transforms;    
    colmap_transforms =  getCameraTranforms(model_path);
    cameraParams colmap_cam_params(getCameraParameters(model_path));
    // cctag osuus
    // cctagin tuottamat tagien sijainnit kuvissa.
    std::map<std::string, boost::ptr_list<cctag::ICCTag>> image_tags;
    image_tags = readTagsFromImages(path);
    std::cout << "total images scanned " << image_tags.size() << "\n";
    filterImagesWithNoTags(image_tags);
    std::cout << "images_with_ some tags " << image_tags.size() << "\n";
    filterReliableTags(image_tags);
    std::cout << "images_with reliable tags " << image_tags.size() << "\n";

    //matikka osuus..
    
    // kameran sijainti tagien suhteen kuvassa x, x2, miten valita x haluaisin sortata colmap virheen kautta
    // mutta per frame täytyy laskea manuaalisesti joten for now on -> use one of the images 
    std::map<std::string, boost::ptr_list<cctag::ICCTag>> many_tags;
    std::map<std::string, boost::ptr_list<cctag::ICCTag>> four_tags;
    std::map<std::string, boost::ptr_list<cctag::ICCTag>> three_tags;
    std::map<std::string, boost::ptr_list<cctag::ICCTag>> two_tags;
    std::map<std::string, boost::ptr_list<cctag::ICCTag>> one_tag;

    for (auto map_it = image_tags.begin(); map_it != image_tags.end(); ) {
        switch (map_it->second.size()) {
            case 0:
                ++map_it;  
                break;
            case 1:
                one_tag.emplace(map_it->first, std::move(map_it->second));
                map_it = image_tags.erase(map_it);
                break;
            case 2:
                two_tags.emplace(map_it->first, std::move(map_it->second));
                map_it = image_tags.erase(map_it);
                break;
            case 3:
                three_tags.emplace(map_it->first, std::move(map_it->second));
                map_it = image_tags.erase(map_it);
                break;
            case 4:
                four_tags.emplace(map_it->first, std::move(map_it->second));
                map_it = image_tags.erase(map_it);
                break;
            default:  // size > 4 or size == 0
                many_tags.emplace(map_it->first, std::move(map_it->second));
                map_it = image_tags.erase(map_it);
                break;
        }
    }


    // let's say we want to solve with 3 tags
    if ( three_tags.size()> 0){
        std::vector<Eigen::Vector3d> world_points;
        world_points.emplace_back(0.0f,  0.0f,  0.0f);  
        world_points.emplace_back(0.105f, 0.0f,  0.0f); 
        world_points.emplace_back(0.0f,  0.184f, 0.0f); 
        
        std::vector<Eigen::Vector3d> imagePoints;
        // for now use the first image
        std::string image_name = three_tags.begin()->first;

        auto &plist = three_tags.begin()->second;
        int num_read = 0;
        for (auto &tag : plist) {
            double x = static_cast<double>(tag.x());
            double y = static_cast<double>(tag.y());
            double z = 0.0;
            imagePoints.emplace_back(Eigen::Vector3d(x, y, z));
            if (num_read > 2) break;
            num_read++;
        };
        std::vector<transform> tag_based_cams =  solve3Tags1Img(world_points,imagePoints,colmap_cam_params.k,colmap_cam_params.distortion);
        for (auto &trans : tag_based_cams){
            std::cout << trans.filename << " was calculated to have rot and trans: \n" 
                << trans.rot << "\n" << trans.translation << "\n";
        }
    }
    else{
        std::cout << "not enough tags found \n";
    }


}

