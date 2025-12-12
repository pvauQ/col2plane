#include "three_p_solve.h"
#include <vector>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/quaternion.hpp>
#include <string>
#include <iostream>
#include <fstream>
void testing(){

    /* this is copypaste fuckery from dataset:
    x, y  id, jos negatiivinen niin ei kelpaa. nää on vain keskukset
    1745.6 2047.55 0 1
    2141.38 2064.5 1 1
    1465.5 2519.01 2 1
    /img Done processing image /media/sf_photodir/mono_jarkkari/_MG_0613.JPG
    */

    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;

    objectPoints.emplace_back(0.0f,  0.0f,  0.0f);  
    objectPoints.emplace_back(0.105f, 0.0f,  0.0f); 
    objectPoints.emplace_back(0.0f,  0.184f, 0.0f); 

    imagePoints.emplace_back(1745.60f, 2047.55f);
    imagePoints.emplace_back(2141.38f, 2064.50f);
    imagePoints.emplace_back(1465.50f, 2519.01f);
    
    // intricts
    /* 1 OPENCV 5184 3456 
    4307.0540120906871 4293.8142903249627 2592 1728 -0.16033441922034594 0.13208610187300843 -0.0011941866158257344 -0.0015545627590278883
    - focal_x, focal_y, principal point 1,2 (u_x,u_y), skew L
			[f_x  L    u_x]
			[0   F_y  u_y ]
			[0    0     1 ]
    
    */
    cv::Mat dist = (cv::Mat_<double>(1,4) << -0.1603, 0.1321, -0.0012, -0.0016); 
    cv::Mat K = (cv::Mat_<double>(3,3) << 4307.054, 0, 2592,
                                        0, 4293.814,  1728,
                                        0,      0,      1);

    std::vector<cv::Mat> rot_out, trans_out;
    std::vector<cv::Mat> matrix_rots;

    int solutions = cv::solveP3P(objectPoints, imagePoints, K,dist, rot_out, trans_out, cv::SOLVEPNP_P3P);
    matrix_rots.resize(solutions);
    std::cout << "Found "  <<solutions <<  " solutions" << std::endl;
    for(int i = 0; i < solutions; i++) {
        cv::Rodrigues(rot_out[i],matrix_rots[i]);
        //std::cout << "Solution " << i << ": trans_vec = \n " << trans_out[i] <<
        //"\n rot:=\n" << matrix_rots[i] << "\n";
    }
    //eli siis nämä solutionit on kameran sijainti suhteessa näihin markkereihin
    /*
    # Image list with two lines of data per image:
    #   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
    #   POINTS2D[] as (X, Y, POINT3D_ID)
    # Number of images: 39, mean observations per image: 814.71794871794873
    */
    //_MG_0613.JPG - colmapin tuottama sijainti?
    // 25 0.72402241353895425 -0.021557127884627592 -0.55850954893523952 -0.40422013640929455 0.92225406585555425 -1.1138989085285935 3.4343808787741223 1 _MG_0613.JPG
    // tämä on siis frame 25

    cv::Quatd col_rot = cv::Quatd(0.72402241353895425, -0.021557127884627592, -0.55850954893523952, -0.40422013640929455);
    cv::Vec3d col_trans = cv::Vec3d(0.92225406585555425, -1.1138989085285935, 3.4343808787741223);

    // nyt meidän tehtävä :-> selvitä muunnos jolla päästään meidän laskemasta tähän colmapin tuottamaan?
    // koska p3p antaa usean ratkaisun meillä on useampi muunnos, for now kokeillaan kaikki?
    cv::Matx33d col_rot_matrix = col_rot.toRotMat3x3();
    //Rrel =Rtar * RcurTranspose target on meidän laskema ja current on mallin luoma...
    cv::transpose(col_rot_matrix,col_rot_matrix);
    cv::transpose(col_trans, col_trans);
    std::vector<cv::Mat> relative_rots;
    std::vector<cv::Vec3d> relative_trans;
    relative_rots.resize(solutions);
    relative_trans.resize(solutions);

    for(int i = 0; i< solutions;i++){
        relative_rots[i] = matrix_rots[i] * col_rot_matrix;

        std::cout << "mahdollinen rotaation muunnos matriisi: \n" << i<< " "<<  relative_rots[i] << "\n";

        // tämä osio vain kikkailee matriisit vektoreiksi open_cv on kiva ja hauska.
        // take first solution from solveP3P
        cv::Mat t = trans_out[i];    // 3x1
        // option 1: make a Vec3f from the Mat
        cv::Vec3d t_vec;
        t_vec[0] = static_cast<double>(t.at<double>(0,0));  // or at<float> if CV_32F
        t_vec[1] = static_cast<double>(t.at<double>(1,0));
        t_vec[2] = static_cast<double>(t.at<double>(2,0));

        // now you can do vector ops
        cv::Vec3d diff = t_vec - col_trans;

        relative_trans[i] =  diff;
        std::cout << "mahdolinen translaatio : \n"  <<   diff << "\n\n";  
    }
    transToFile(col_rot, relative_trans[0]);
    
    std::cout << "done testing \n";
}

//The matrix.txt file is expected to contain scale qw qx qy qz tx ty tz and not the 4x4 transformation matrix. See Sim3d::FromFile.
void transToFile (cv::Quatd rotation, cv::Vec3d trans){
    std::string file = "transformation.txt";
    std::ofstream out_stream(file);
    out_stream << std::setprecision(std::numeric_limits<double>::digits10 + 2);
    out_stream << 1 << " " 
               << rotation[0] << " " 
               << rotation[1] << " " 
               << rotation[2] << " " 
               << rotation[3] << " " 
               << trans[0]    << " " 
               << trans[1]    << " " 
               << trans[2]    << "\n "; 



}

void transformPoints(cv::Vec3d translation){

    //# 3D point list with one line of data per point:
    //POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)'
    //Number of points: 7793, mean track length: 4.0772488130373414
    // 1 2.5595058284510146 2.6611118648925518 2.7027644118760135 144 134 109 0.43505140914488483 6 1161 7 1292 8 2159 9 2441

    std::string file = "points3D.txt";
    std::string output_file = "transformed points3D.txt";
    std::ifstream input_stream(file);
    std::ofstream output_stream(output_file);
    output_stream << std::setprecision(std::numeric_limits<double>::digits10 + 2);

    std::string in_str;

    while (getline(input_stream,in_str )) {
        if (in_str.front() == '#'){
            std::stringstream st(in_str);
            std::string rest;
            std::getline(st,rest);
            output_stream << rest  << "\n" ;
            continue;
        }
        std::stringstream st(in_str);
        int id;
        double a, b, c;
        st >> id >> a >> b >> c;
        cv::Vec3d orginal(a,b,c);
        cv::Vec3d tranformed =  orginal - translation;

        std::string rest;
        std::getline(st, rest);
        output_stream  << id << " "
                        << tranformed[0] << " "
                        << tranformed[1] <<  " "
                        << tranformed[2] << ""
                        << rest << "\n";

    }
    if (input_stream.bad()) {
        // IO error
    } 
    
}

void transformCameras(){ // images

    //# Image list with two lines of data per image:
    //#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
    //#   POINTS2D[] as (X, Y, POINT3D_ID)
    //# Number of images: 39, mean observations per image: 814.71794871794873

}


cv::Matx44f genModelMatrix(const cv::Vec3d& translation, const cv::Mat& rotation)
{
    CV_Assert(rotation.rows == 3 && rotation.cols == 3);

    cv::Matx44f M = cv::Matx44f::eye();

    // Copy rotation (cast to float if needed)
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            M(r, c) = static_cast<float>(rotation.at<double>(r, c));
        }
    }

    // Set translation
    M(0, 3) = static_cast<float>(translation[0]);
    M(1, 3) = static_cast<float>(translation[1]);
    M(2, 3) = static_cast<float>(translation[2]);

    return M;
}

void saveTransformTxt(const cv::Matx44f& M, const std::string& filename) {
    std::ofstream file(filename);
    file << std::fixed << std::setprecision(6);
    
    // Row-major: M(row, col) for row=0..3, col=0..3
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            file << M(r, c) << "\n";
        }
    }
    file.close();
}