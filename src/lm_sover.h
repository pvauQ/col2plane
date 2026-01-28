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

s = colmap frame
w = world frame

R rot (s->W)    angle_axis      ?
cam_center (s)  vec3            !
trans ( s- w)   vec3            ?
ray_dir  (s)    vec3 normalized !
ray dist        scalar          ?
X_target ( w)   vec3            !

equation
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
        
        //angle_ax[0] = 1.0;angle_ax[1] = 1.0;angle_ax[2] = 1.0;
        Eigen::AngleAxisd R_aa(angle_ax.norm(), angle_ax.normalized());                
        Eigen::Matrix3d ROT = R_aa.toRotationMatrix();
        
        //std::cout << "cam_c range: " << (cam_c.back()-cam_c[0]).norm() << std::endl;
        //std::cout << "ray_dir dot products: " << ray_dir[0].dot(ray_dir[2]) << std::endl;
        
        //std::cout << "params: " << params.transpose() << std::endl;
        //std::cout << "n_cams: " << n_cams_ << " cam_c.size(): " << cam_c.size() << std::endl;
        //std::cout << "scales: " << scales.transpose() << std::endl;

        //std::cout << "Cam0-target dist: " << (X_target - cam_c[0]).norm() << std::endl;
        //std::cout << "Baseline cam0-cam1: " << (cam_c[0] - cam_c[1]).norm() << std::endl; 
        //std::cout << "Ray lengths: " << ray_dir[0].norm() << " " << ray_dir[1].norm() << std::endl;


        
        // 1 pred = =ROT* cam_c[0] + trans + scales[0]* ROT * ray_dir[0];
        Eigen::Vector3d pred;
        residuals.resize(3 * n_cams_);
        for(int i = 1; i< cam_c.size(); i++){
            if (i ==0){
                pred =ROT* cam_c[i] + trans + 1.0 * ROT * ray_dir[i].normalized(); // lock first ray len so we have unique solution ( this could be anything)
            }
            else{
                pred =ROT* cam_c[i] + trans + scales[i]* ROT * ray_dir[i].normalized();
            }
            residuals.segment<3>(i * 3) = pred - X_target;
        }
        
        std::cout << " error: "<<  sqrt(residuals.squaredNorm() / residuals.size()) << "\n";
        // 1. Decode params → R,t,;
        // 2. Compute predictions R*C_i + t + s_i*R*d_i
        // 3. residuals = predictions - X_target
        return 0;
    }


      typedef double Scalar;
    enum { InputsAtCompileTime = Eigen::Dynamic, ValuesAtCompileTime = Eigen::Dynamic };
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    int inputs() const { return 6 + n_cams_; }
    int values() const { return 3 * n_cams_; }
    
private:
    std::vector<Eigen::Vector3d> cam_c;  // Camera centers
    std::vector<Eigen::Vector3d> ray_dir;  // Ray directions  
    Eigen::Vector3d X_target;        // Target point
    int n_cams_;
};

 Eigen::VectorXd lmDriver( std::vector<Eigen::Vector3d> cams, 
                std::vector<Eigen::Vector3d> ray_dirs,
                Eigen::Vector3d world_pos){
    
    //std::cout << "lm driver has these cam centrums \n";
    //for(const auto& cam: cams){
    //    std::cout << cam << "\n \n";}


    MyFunctor functor(cams,ray_dirs,world_pos);
    Eigen::NumericalDiff<MyFunctor> numDiff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<MyFunctor>, double> solver(numDiff);


    //std::cout << "solver has cameras" << cams[0] << "\n" << cams[1] << "\n";
    //solver.parameters.ftol = 1e-8;    
    //solver.parameters.xtol = 1e-8;     
    //solver.parameters.maxfev = 100; 

    solver.parameters.ftol = 1e-4;     
    solver.parameters.xtol = 1e-6;      
    solver.parameters.gtol = 1e-4;
    solver.parameters.maxfev = 5000; 

    Eigen::Vector3d is = (cams[0]+cams[1]+ cams[2]) / 3.0;
    Eigen::VectorXd params(6 +cams.size());

    
    params << 0.0, 0.0, 0.0,  // Rotation vector (identity initially)
              is[0], is[1], is[2],  // Translation
              Eigen::VectorXd::Ones(cams.size());

    Eigen::AngleAxisd init_rot(55.0 * M_PI/180.0, Eigen::Vector3d(0,1,0));
    params.segment<3>(0) = init_rot.axis() * init_rot.angle();
    
    solver.minimize(params);

    //std::cout << params.segment<3>(3) <<"\n";
    //std::cout << " after solve" "\n";
    //std::cout <<" rots"  << params.segment<3>(0)   << "\n";
    //std::cout <<" trans"  << params.segment<3>(3)   << "\n";
    //std::cout <<" depths"  << params.segment(6, cams.size())   << "\n";

    return params;
    // https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm
}



#endif