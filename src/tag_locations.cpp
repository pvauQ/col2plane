#include "tag_locations.h"
#include <eigen3/Eigen/Geometry>
#include <cctag/CCTag.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry>
#include <filesystem>
#include <map>
// etsitään kuvat joissa tägejä näkyy n kappaletta
//  näistä löydetyistä kuvista halutaan ne joissa on pisteitä joista pitkä track



void readTagsFromImages(std::filesystem::path dir){
	const std::size_t nCrowns{3};
	cctag::Parameters params(nCrowns);
	params.setUseCuda(false);
    std::map<std::string, std::vector<std::unique_ptr<cctag::ICCTag>>> image_markers;

    try {
        for (const auto& entry : std::filesystem::directory_iterator(dir)) {
            if (entry.is_regular_file()) {
                // todo check that it's actually image..
                //std::cout << entry.path() << '\n'; 
                cv::Mat im = cv::imread(entry.path());
                cv::cvtColor(im,im,cv::COLOR_BGR2GRAY);
                
                const int pipeId{0};
                // an arbitrary id fo
                const int frameId{0};
                boost::ptr_list<cctag::ICCTag> boost_markers{};
                cctagDetection(boost_markers, pipeId, frameId, im, params);
                //cctag is built using boost pointers. I do not want those in my project so let's partyy
                std::vector<std::unique_ptr<cctag::ICCTag>> markers;
                markers.reserve(boost_markers.size());  // avoid reallocs


                for (auto it = boost_markers.begin(); it != boost_markers.end(); ++it) {
                    // Assuming ICCTag has a clone() method that returns a pointer to a copy
                    markers.emplace_back(it->clone());
                }
                image_markers.emplace(entry.path().filename().string(), std::move(markers));
                std::cout << "image " << entry.path().filename().string() << "has " << markers.size() << "markers\n";
            }
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error: " << e.what() << '\n';
    }




}