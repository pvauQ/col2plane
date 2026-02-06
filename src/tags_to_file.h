#ifndef TAGS_TO_FILE_H
#define TAGS_TO_FILE_H
#include <filesystem>
#include <cctag/CCTag.hpp>



namespace CctagFileHelper{

struct tagInfo{ // this tells what tags are visible in what image and where tey are
    std::string image_name;
    std::vector<std::pair<double,double>> coordinates;
    std::vector<int> ids; // tag ids
    // ellipse stuff


};

std::vector<tagInfo> readTagsFromImages(std::filesystem::path dir);
std::vector<tagInfo> readTagsFromtxt(std::filesystem::path path);
void saveTagsToFile(std::filesystem::path dir, std::vector<tagInfo> tags);
std::map<int,Eigen::Vector3d> TagWorldLocationFromFile(std::filesystem::path path);

};
#endif