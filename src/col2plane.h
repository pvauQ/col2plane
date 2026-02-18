#ifndef COLTOPLANE_H
#define COLTOPLANE_H
#include <filesystem>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <map>
#include "tags_to_file.h"
#include "tag_image_solver.h"
#include "colmap_stuff.h"

enum solve_mode {
  P3P_SOLVE,
  LM_SOLVE,
  ELLIPSE_SOLVE
}; 

//pair that has info of the tag and camera that sees it
struct tag_col_dir
{
    CctagFileHelper::tagInfo tag_info;
    transform camera_info;
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

    private: 
    Eigen::Vector3d  calculateCameraRay(double x,  double y, transform camera);
    std::vector<transform> colmap_transforms;  
    cameraParams colmap_cam_params;

    void CollectCamRays(std::vector<Eigen::Vector3d> & cams, std::vector<Eigen::Vector3d> & ray_dirs, std::vector<tag_col_dir> use_for_solve, int tag_to_use, int n_cams_to_use);
     std::vector<tag_col_dir> filterAndSortByTag(const std::vector<tag_col_dir>& image_infos, int tag_id);
};


std::vector<matrixTransform> FilterBestN(std::vector<matrixTransform>   i_transforms , size_t num_out);
std::vector<matrixTransform> FilterByError(std::vector<matrixTransform> &  i_transforms , double max_error);

#endif