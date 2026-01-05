
#include "tags_to_file.h"
#include <filesystem>
#include <cctag/CCTag.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <utility>
#include <assert.h>



namespace CctagFileHelper{


std::vector<tagInfo> readTagsFromImages(std::filesystem::path dir){

    const std::size_t nCrowns{3};
    cctag::Parameters params(nCrowns);
    params.setUseCuda(true);
    std::map<std::string, boost::ptr_list<cctag::ICCTag>> image_markers;
    try {
        int frame = 0;
        for (const auto& entry : std::filesystem::directory_iterator(dir)) {
            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                if (!(ext == ".jpg" | ext == ".jpeg" | ext == ".png"| ext == "bmp")){
                    continue;
                }
                cv::Mat im = cv::imread(entry.path());
                cv::cvtColor(im,im,cv::COLOR_BGR2GRAY);
                
                const int pipeId{0};
                // an arbitrary id fo
                const int frameId{frame};
                frame++;
                boost::ptr_list<cctag::ICCTag> boost_markers{};
                cctagDetection(boost_markers, pipeId, frameId, im, params);
                // TODO: get away from boost stuf..
                image_markers.emplace(entry.path().filename().string(), boost_markers);
                std::cout << "" << entry.path().filename().string() << " : " << boost_markers.size()  << "\n";//care this is "possible tags " not marked correct
                }
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error: " << e.what() << '\n';
    }
    std::vector<tagInfo> tags;

    for(const auto& entry : image_markers){
        tagInfo info;
        
        info.image_name = entry.first;
        for (const auto& tag : entry.second){
            if (tag.getStatus() == cctag::status::id_reliable){
                info.coordinates.push_back({tag.x(), tag.y()});
                info.ids.push_back(tag.id());
                // todo: ellipse stuff
            }
        }
        tags.push_back(info);
    }
    saveTagsToFile(std::filesystem::current_path(), tags);
    return tags;
}



void saveTagsToFile(std::filesystem::path dir, std::vector<tagInfo> tags){

    std::ofstream out_stream(dir / "tags.txt");
    if (!out_stream.is_open()) {
        throw std::runtime_error("Failed to open tags.txt");
    }
    out_stream << "# " << tags.size() << " images" << "\n";
    out_stream << std::fixed << std::setprecision(8);
    
    for ( const tagInfo& im : tags){  
        out_stream << im.image_name << " ";
        assert(im.ids.size() == im.coordinates.size());
        for (int i = 0; i <im.ids.size(); i++){
            out_stream << im.coordinates[i].first << " "<< im.coordinates[i].second << " " << im.ids[i] << " ";
        }
        out_stream << "\n";
    }
}


std::vector<tagInfo> readTagsFromtxt(std::filesystem::path path) {
    std::vector<tagInfo> tags;
    std::ifstream in_stream(path);
    if (!in_stream.is_open()) {
        throw std::runtime_error("Failed to open tags.txt");
    }
    std::string line;
    std::getline(in_stream, line);
    
    while (std::getline(in_stream, line)) {
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        tagInfo info;
        iss >> info.image_name;
        
        double x, y;
        int id;
        while (iss >> x >> y >> id) {
            info.coordinates.emplace_back(x, y);
            info.ids.push_back(id);
        }       
        assert(info.coordinates.size() == info.ids.size());
        tags.push_back(std::move(info));
    }
    return tags;
}
}