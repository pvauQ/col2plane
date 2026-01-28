#ifndef LM_SOLVER_H
#define LM_SOLVER_H
 #include <iostream>
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
private:
    std::vector<Eigen::Vector3d> cams1, ray_dirs1, cams2, ray_dirs2, X_targets;
    size_t n_cams_;
    
public:
    typedef double Scalar;
    enum { InputsAtCompileTime = Eigen::Dynamic, ValuesAtCompileTime = Eigen::Dynamic };
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    int inputs() const { return 6 + n_cams_; }
    int values() const { return 3 * n_cams_; }


    MyFunctor(const std::vector<Eigen::Vector3d>& cams1, 
              const std::vector<Eigen::Vector3d>& ray_dirs1,
              const std::vector<Eigen::Vector3d>& cams2,
              const std::vector<Eigen::Vector3d>& ray_dirs2, 
              const std::vector<Eigen::Vector3d>& X_targets) 
         : cams1(cams1), 
           ray_dirs1(ray_dirs1), 
           cams2(cams2), 
           ray_dirs2(ray_dirs2), 
           X_targets(X_targets),
           n_cams_(cams1.size() + cams2.size() ){
            //TODO:: ASSERT NÄMÄ VIDDU
            assert (cams1.size() == ray_dirs1.size());
            assert (cams2.size() == ray_dirs2.size());  // all cameras need to have ray_dir
           }
    

    ///params = rot ( angle_Axis),trans(vec3), ray_distances(1 for each cam vec3?)
    int operator()(const Eigen::VectorXd& params,  // from s ->w
                     Eigen::VectorXd& residuals) const {
        
        Eigen::Vector3d angle_ax = params.segment<3>(0);
        Eigen::Vector3d trans    = params.segment<3>(3);
        Eigen::VectorXd scales   = params.segment(6,n_cams_);
        
        //angle_ax[0] = 1.0;angle_ax[1] = 1.0;angle_ax[2] = 1.0;
        Eigen::AngleAxisd R_aa(angle_ax.norm(), angle_ax.normalized());                
        Eigen::Matrix3d ROT = R_aa.toRotationMatrix();

        
        // 1 pred = =ROT* cam_c[0] + trans + scales[0]* ROT * ray_dir[0];
        Eigen::Vector3d pred0;
        Eigen::Vector3d pred1;
        //residuals.resize(6 * n_cams_);
        assert(cams1.size()  <= cams2.size()); //TODO: make work for any sizes.
        for(int i = 0; i< cams1.size(); i++){
            pred0 =ROT* cams1[i] + trans + scales[i]* ROT * ray_dirs1[i];
            pred1 =ROT* cams2[i] + trans + scales[i+ cams1.size()]* ROT * ray_dirs2[i];
            
            //if (i == 0){
            //    pred0 =ROT* cams1[i] + trans + 1.0* ROT * ray_dirs1[i];
            //}
        
            residuals.segment<3>(i * 6 + 0) = pred0 - X_targets[0];
            residuals.segment<3>(i * 6 + 3) = pred1 - X_targets[1];

        }
        
        std::cout << " error: "<<  sqrt(residuals.squaredNorm() / residuals.size()) << "\n";
        // 1. Decode params → R,t,;
        // 2. Compute predictions R*C_i + t + s_i*R*d_i
        // 3. residuals = predictions - X_target
        return 0;
    }
};



 Eigen::VectorXd lmDriver( std::vector<Eigen::Vector3d> cams1, 
                std::vector<Eigen::Vector3d> ray_dirs1,
                std::vector<Eigen::Vector3d> cams2, 
                std::vector<Eigen::Vector3d> ray_dirs2,
                std::vector<Eigen::Vector3d> world_pos){
    
    //std::cout << "lm driver has these cam centrums \n";
    //for(const auto& cam: cams){
    //    std::cout << cam << "\n \n";}


    MyFunctor functor(cams1,ray_dirs1, cams2,ray_dirs2,  world_pos);
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
    
    int total_cams = cams1.size() + cams2.size();
    Eigen::Vector3d is = (cams1[0]+cams1[1]+ cams1[2]) / 3.0; // something to use as initial
    Eigen::VectorXd params(6 + total_cams);
    
    params << 0.0, 0.0, 0.0,  // Rotation vector (identity initially)
              is[0], is[1], is[2],  // Translation
              Eigen::VectorXd::Ones(total_cams);
    params.segment<1>(6).setConstant(1.0); 

    Eigen::AngleAxisd init_rot(35.0 * M_PI/180.0, Eigen::Vector3d(0,1,0)); // just something initially
    params.segment<3>(0) = init_rot.axis() * init_rot.angle();
    
    solver.minimize(params);

    std::cout << params.segment<3>(3) <<"\n";
    std::cout << " after solve" "\n";
    std::cout <<" rots"  << params.segment<3>(0)   << "\n";
    std::cout <<" trans"  << params.segment<3>(3)   << "\n";
    std::cout <<" depths"  << params.segment(6, total_cams)  << "\n";

    return params;
    // https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm
}



#endif