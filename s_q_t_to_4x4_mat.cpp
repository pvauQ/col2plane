
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Geometry>

int main(int argc, char* argv[])
{
    std::string i_file = argv[1];
    std::string o_file = argv[2];
    std::ifstream in(i_file);
    std::ofstream out(o_file);
    std::cout << i_file << "  in \n";

    float scale, qw, qx, qy, qz, tx, ty, tz;

    in >>  scale >> qw >> qx >> qy >> qz >> tx >> ty >> tz;

    std::cout << scale << "\n";
    Eigen::Quaternionf q(qw,qx,qy,qz);

    Eigen::Matrix4f M = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f ROT = q.toRotationMatrix();
    Eigen::Vector3f t(tx,ty,tz);
    M.block<3,3>(0,0) = scale * ROT;
    M.block<3,1>(0,3) = t;
    std::cout << "koiran perse \n";

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out << M(i,j);
            if (j < 3) out << " ";
        }
    out << "\n";
}
}