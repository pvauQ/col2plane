#ifndef COLTOPLANE_H
#define COLTOPLANE_H
#include <filesystem>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <map>
#include "tags_to_file.h"
#include "tag_image_solver.h"


enum solve_mode {
  PP3_SOLVE,
  LM_SOLVE,
  ELLIPSE_SOLVE
}; 

class Col2Plane
{
    
    public:
    Col2Plane();

    void withPrecalulated(std::string file);
    void calcCctag();

    void printTagInfos();

    void cameraPosOutput(std::vector<matrixTransform>& trans );

    void col2CctSpace(solve_mode mode);
    void kissa();

    std::vector<CctagFileHelper::tagInfo> img_tags;
    std::map<int,Eigen::Vector3d> marker_word_pos;
};
std::vector<matrixTransform> FilterBestN(std::vector<matrixTransform>   i_transforms , size_t num_out);
std::vector<matrixTransform> FilterByError(std::vector<matrixTransform> &  i_transforms , double max_error);

#endif