#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry>
#include <filesystem>
#include <iostream>
#include <algorithm>
#include "col2plane.h"
#include "three_p_solve.h"
#include "tag_image_solver.h"
#include "tag_locations.h"
#include "colmap_stuff.h"
#include "tags_to_file.h"



Col2Plane::Col2Plane(){
    //set marker world positions;
    // asume wrong order as printed clocwise starting from upper left -> 3,1,2,0
    this->marker_word_pos.emplace(0,Eigen::Vector3d(0.0,  0.0,  0.0));
    this->marker_word_pos.emplace(3,Eigen::Vector3d(0.0,  0.121,  0.0));
    this->marker_word_pos.emplace(1,Eigen::Vector3d(0.195,  0.121,  0.0));
    this->marker_word_pos.emplace(2,Eigen::Vector3d(0.195,  0.0,  0.0));
    
}


// cctag output previously saved to a file
void Col2Plane::withPrecalulated(std::string file){
    std::filesystem::path p = std::filesystem::current_path();
    tags = CctagFileHelper::readTagsFromtxt(p/file);
    this->printTagInfos();
}

void Col2Plane::calcCctag(){

    std::filesystem::path path("photodir/malli");
    std::filesystem::path model_path  =path ;
    tags = CctagFileHelper::readTagsFromImages(path);
    this->printTagInfos();

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

// number of tags to use for transformation, this will determine the  operation method
// 4 -> nice solution
// 3 -> nice solution but require multiple images
void Col2Plane::col2CctSpace(int num_tags){

    std::filesystem::path path("photodir/malli");
    std::filesystem::path model_path  =path ;
    // colmapin tuottamat kamerat ja näiden transformit
    std::vector<transform> colmap_transforms;    
    colmap_transforms =  getCameraTranforms(model_path);
    cameraParams colmap_cam_params(getCameraParameters(model_path));


    std::vector<CctagFileHelper::tagInfo>  ims_to_use;
    std::vector<matrixTransform> tag_based_cams;
    std::vector<Eigen::Vector3d> world_pos;
    std::vector<Eigen::Vector3d> image_pos;
    CctagFileHelper::tagInfo im;
    switch (num_tags)
    {
    case 4:
        for(const auto& im: tags){
            if (im.ids.size() == 4)
            ims_to_use.push_back(im);
        }
        std::cout << ims_to_use.size() << " images with 4 tags ";
        im=  ims_to_use.front();
        std::cout << "using " << im.image_name << "\n ";



        for(int i = 0 ; i< im.ids.size(); i++){
            int id = im.ids[i];
            world_pos.push_back(marker_word_pos.find(id)->second);
            Eigen::Vector3d im_pos = Eigen::Vector3d(im.coordinates[i].first, im.coordinates[i].second, 0.0);
            image_pos.push_back(im_pos);;
            // note -> onko koordinaatit nyt oikeassa järjestyksess' ( x , y ,z )
        }
        tag_based_cams =  solve3Tags1Img(world_pos, image_pos,colmap_cam_params.k, colmap_cam_params.distortion);

        break;
    
    default:
        break;
    }


    return;

}


void kissa(){

    
    std::filesystem::path path("photodir/malli");
    std::filesystem::path model_path  =path ;
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


    // let's say we want to solve with 4 tags
    if ( four_tags.size()> 0){
        std::vector<Eigen::Vector3d> world_points;
        // kolme pistettä pysty
        //world_points.emplace_back(0.0,  0.0,  0.0);  
        //world_points.emplace_back(0.105, 0.0 ,  0.0); 
        //world_points.emplace_back(0.0,  -0.0, 0.184); 
        //neljä pistettä vaaka
        world_points.emplace_back(0.0,  0.0,  0.0);
        world_points.emplace_back(0.195,  0.0,  0.0);
        world_points.emplace_back(0.195,  0.0,  -0.121);
        world_points.emplace_back(0.0,  0.0,  -0.121);


        std::vector<Eigen::Vector3d> imagePoints;
        // for now use the first image
        std::string image_name = four_tags.begin()->first;

        // make sure that tags found in images are in same order as defined in world points
        auto &plist = four_tags.begin()->second;
        plist.sort([](cctag::ICCTag const& a, cctag::ICCTag const& b) {
             return a.id() < b.id();
        });


        int num_read = 0;
        std::cout << "solving using image: " << image_name << " tags used in";
        for (auto &tag : plist) {
            std::cout << " " << tag.id() ;
            double x = static_cast<double>(tag.x());
            double y = static_cast<double>(tag.y());
            double z = 0.0;
            imagePoints.emplace_back(Eigen::Vector3d(x, y, z));
            if (num_read > 4 ) break;
            num_read++;
        };
        std::cout<< std::endl;

        std::vector<matrixTransform> tag_based_cams =  solve3Tags1Img(world_points,imagePoints,colmap_cam_params.k,colmap_cam_params.distortion);
        for (auto &trans : tag_based_cams){
            std::cout << " transform was calculated to have location defined by: \n" 
                << trans.rotation << "\n" << trans.location << "\n";
        }

        auto it = std::find_if(colmap_transforms.begin(), colmap_transforms.end(), 
                            [image_name](transform x) { return x.filename == image_name; });
        transform col_cam_trans = *it;
        assert(col_cam_trans.filename == image_name);
        matrixTransform col_cam_m_trans;

        Eigen::Matrix3d col_rot = col_cam_trans.rot.toRotationMatrix();
        Eigen::Vector3d col_trans = col_cam_trans.translation;
        // nyt meillä on cctag kameran sijainti, ja colmap kameran sijainti
        // tekoäly sanoo seuraavaa ->  nämä on periaateessa world to camera
        // ja jotta tässä diffissä on järkeä täytyy muunnos tapahtua cam-> world

        std::vector<matrixTransform> possible_solutions;
        for (auto &trans : tag_based_cams){
            //
            Eigen::Matrix3d R_2world_colmap = col_rot.transpose();
            Eigen::Vector3d t_2world_colmap = -col_rot.transpose() * col_trans;
            Eigen::Matrix3d R_2world_p3p = trans.rotation.transpose();
            Eigen::Vector3d t_2world_p3p = -trans.rotation.transpose() * trans.location;


            Eigen::Matrix3d rotation_dif = R_2world_p3p *R_2world_colmap.transpose();
            Eigen::Vector3d transpose_dif = t_2world_p3p - rotation_dif * t_2world_colmap;
            matrixTransform sol;
            sol.rotation = rotation_dif;
            sol.location = transpose_dif;
            possible_solutions.push_back(sol);
        }
        // nyt meillä on ratkaisut jotka siirtävät colmap kameran cctag kameran sijaintiin

        // mennään quaternion + trans vector maailmaan koska colmap haluaa semmoisen;
        int ind = 0;
        for (auto &sol: possible_solutions){
            transToFile(Eigen::Quaterniond(sol.rotation), sol.location, std::to_string(ind));
            ind++;
        }

        // etsitään se
        // ->  rot + trans
        // ratkaistaan rotaatio joka siitää colmap kameran cctag kameraan
        // /M+t) * colmap M,t = cctag M t


    }
    else{
        std::cout << "not enough tags found \n";
    }


}
