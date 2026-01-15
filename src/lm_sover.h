#ifndef LM_SOLVER_H
#define LM_SOLVER_H
#include <Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <string>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

/*
s = colmap mallin koordinaatisto
w = maailman koordinaatisto ( defined via tag locations)

R rot (s->W)    angle_axis      ?
cam_center (s)  vec3            !
trans ( s- w)   vec3            ?
ray_dir  (s)    vec3 normalized !
ray dist        scalar          ?
X_target ( w)   vec3            !

yhtälö
R*cam_center +trans + ray_dist*R*ray_dir = X_target


Rot ja trans ratkaisu mahdollistaa  muunnoksen s->w
*/

class MyFunctor {
public:
    MyFunctor(const std::vector<Eigen::Vector3d>& cam_c, 
              const std::vector<Eigen::Vector3d>& ray_dir, 
              const Eigen::Vector3d& X_target) 
         : cam_c(cam_c), 
           ray_dir(ray_dir), 
           X_target(X_target),
           n_cams_(cam_c.size()){
            //TODO:: ASSERT NÄMÄ VIDDU
            assert (cam_c.size() == ray_dir.size()); // all cameras need to have ray_dir
           }
    
    ///params = rot ( angle_Axis),trans(vec3), ray_distances(1 for each cam vec3?)
    int operator()(const Eigen::VectorXd& params,  // from s ->w
                     Eigen::VectorXd& residuals) const {
        
        Eigen::Vector3d angle_ax = params.segment<3>(0);
        Eigen::Vector3d trans    = params.segment<3>(3);
        Eigen::VectorXd scales   = params.segment(6, n_cams_);
        
        Eigen::AngleAxisd R_aa(angle_ax.norm(), angle_ax.normalized());                
        Eigen::Matrix3d ROT = R_aa.toRotationMatrix();
        
        
        // 1 pred = =ROT* cam_c[0] + trans + scales[0]* ROT * ray_dir[0];
        Eigen::Vector3d pred;
        residuals.resize(3 * n_cams_);
        for(int i = 0; i< cam_c.size(); i++){
            pred =ROT* cam_c[i] + trans + scales[i]* ROT * ray_dir[i];
            residuals.segment<3>(i * 3) = pred - X_target;
        }
        

        // 1. Decode params → R,t,
        // 2. Compute predictions R*C_i + t + s_i*R*d_i
        // 3. residuals = predictions - X_target
        return 0;
    }


      typedef double Scalar;
    enum { InputsAtCompileTime = Eigen::Dynamic, ValuesAtCompileTime = Eigen::Dynamic };
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    int inputs() const { return 6 + n_cams_; }  // ALWAYS 9 for your problem
    int values() const { return 3 * n_cams_; }  // 3 residuals per camera
    //int values() const { return 9; }  // ALWAYS 9
    
private:
    std::vector<Eigen::Vector3d> cam_c;  // Camera centers
    std::vector<Eigen::Vector3d> ray_dir;  // Ray directions  
    Eigen::Vector3d X_target;        // Target point
    int n_cams_;
};

void lmDriver( std::vector<Eigen::Vector3d> cams, 
                std::vector<Eigen::Vector3d> ray_dirs,
                Eigen::Vector3d world_pos){
    
    //std::cout << "lm driver has these cam centrums \n";
    //for(const auto& cam: cams){
    //    std::cout << cam << "\n \n";}


    MyFunctor functor(cams,ray_dirs,world_pos);
    Eigen::NumericalDiff<MyFunctor> numDiff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<MyFunctor>, double> solver(numDiff);



    //solver.maxIterations = 1000;
    solver.iter = 100000;
    Eigen::VectorXd params(6 +cams.size());
    params << 0.0, 0.0, 0.0,  // Rotation vector (identity initially)
              2.0, 3.0, 4.0,  // Translation
              Eigen::VectorXd::Ones(cams.size());

    std::cout << params.segment<3>(3) <<"\n";
    solver.minimize(params);
    std::cout << " after solve" "\n";
    std::cout << params.segment<3>(3)   << "\n";
    std::cout <<" depths"  << params.segment<3>(0)   << "\n";
    

    // solverin parametrit vakoile= https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm
}



#endif