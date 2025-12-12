#ifndef colmapstuff
#define colmapstuff
#include <filesystem>
#include <cctag/CCTag.hpp>
#include "tag_image_solver.h"

std::vector<transform> getCameraTranforms(std::filesystem::path colmap_model_dir);

#endif
