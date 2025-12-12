#ifndef THREEPOINTSOLVE
#define THREEPOINTSOLVE
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/quaternion.hpp>


/* this file is for testing stuff..*/

void testing();
void transformPoints(cv::Vec3d translation);
void transformCameras();

void transToFile (cv::Quatd rotation, cv::Vec3d trans);

cv::Matx44f genModelMatrix(const cv::Vec3d& translation, const cv::Mat& rotation);

#endif