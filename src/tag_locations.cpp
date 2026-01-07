#include "tag_locations.h"
#include <eigen3/Eigen/Geometry>
#include <cctag/CCTag.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry>
#include <filesystem>
#include <map>
#include "tag_image_solver.h"



//returns map with str filename -> boost::ptr_list<cctag::ICCTag>> 
// this does the stuff related to cctag
std::map<std::string, boost::ptr_list<cctag::ICCTag>> readTagsFromImages(std::filesystem::path dir){

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
                std::cout << "image " << entry.path().filename().string() << " has " << boost_markers.size() << " possible markers\n";
                }
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error: " << e.what() << '\n';
    }
    return image_markers;
}

