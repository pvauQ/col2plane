#ifndef COLTOPLANE_H
#define COLTOPLANE_H
#include <filesystem>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <map>
#include "tags_to_file.h"
#include "tag_image_solver.h"

class Col2Plane
{
    
    public:
    Col2Plane();

    void withPrecalulated(std::string file);
    void calcCctag();

    void printTagInfos();

    void cameraPosOutput(std::vector<matrixTransform>& trans );

    void col2CctSpace(int num_tags);
    void kissa();

    std::vector<CctagFileHelper::tagInfo> tags;
    std::map<int,Eigen::Vector3d> marker_word_pos;
};

#endif