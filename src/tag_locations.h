#ifndef TAG_lOCATIONS_H
#define TAG_lOCATIONS_H
#include <opencv4/opencv2/opencv.hpp>
#include <filesystem>
#include <cctag/CCTag.hpp>




std::map<std::string, boost::ptr_list<cctag::ICCTag>> readTagsFromImages(std::filesystem::path dir);

#endif