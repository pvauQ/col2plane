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
solving  = ?
known = !

s = intial frame
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
    std::vector<Eigen::Vector3d> cams1, ray_dirs1, cams2, ray_dirs2, cams3, ray_dirs3, X_targets;
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
              const std::vector<Eigen::Vector3d>& cams3,
              const std::vector<Eigen::Vector3d>& ray_dirs3,
              const std::vector<Eigen::Vector3d>& X_targets) 
         : cams1(cams1), 
           ray_dirs1(ray_dirs1), 
           cams2(cams2), 
           ray_dirs2(ray_dirs2), 
           cams3(cams3), 
           ray_dirs3(ray_dirs3), 
           X_targets(X_targets),
           n_cams_(cams1.size() + cams2.size()+cams3.size() ){
        //TODO:: ASSERT NÄMÄ VIDDU
        assert (cams1.size() == ray_dirs1.size());
        assert (cams2.size() == ray_dirs2.size());  // all cameras need to have ray_dir
        assert (cams3.size() == ray_dirs3.size());
        }
    

    int operator()(const Eigen::VectorXd& params,  // from s ->w
                     Eigen::VectorXd& residuals) const {
        
        Eigen::Vector3d angle_ax = params.segment<3>(0);
        Eigen::Vector3d trans    = params.segment<3>(3);
        Eigen::VectorXd scales   = params.segment(6,n_cams_);
        //angle_ax[1] = 0.0;
        
        //angle_ax[0] = 1.0;angle_ax[1] = 1.0;angle_ax[2] = 1.0;
        Eigen::AngleAxisd R_aa(angle_ax.norm(), angle_ax.normalized());                
        Eigen::Matrix3d ROT = R_aa.toRotationMatrix();

        
        // 1 pred = =ROT* cam_c[0] + trans + scales[0]* ROT * ray_dir[0];
        Eigen::Vector3d pred0;
        Eigen::Vector3d pred1;
        Eigen::Vector3d pred2;
        residuals.resize(3* n_cams_);
        //assert(cams1.size()  <= cams2.size()); //TODO: make work for any sizes.
        for(int i = 0; i< cams1.size(); i++){
            pred0 =ROT* cams1[i] + trans + scales[i]* ROT * ray_dirs1[i];
            pred1 =ROT* cams2[i] + trans + scales[i+ cams1.size()]  * ROT * ray_dirs2[i];
            pred2 =ROT* cams3[i] + trans + scales[i+ cams1.size()+cams2.size()] * ROT * ray_dirs3[i];
            
            if (i == 0){
                //pred0 =ROT* cams1[i] + trans + 0.1* ROT * ray_dirs1[i];
                //pred1 =ROT* cams2[i] + trans + 1.0* ROT * ray_dirs2[i];
                //pred2 =ROT* cams3[i] + trans + 1.0 * ROT * ray_dirs3[i];
            }
        
            residuals.segment<3>(i * 9 + 0) = pred0 - X_targets[0];
            residuals.segment<3>(i * 9 + 3) = pred1 - X_targets[1];
            residuals.segment<3>(i * 9 + 6) = pred2 - X_targets[2];
            

        }
        //std::cout << residuals.transpose()<< "\n";
        //std::cout << " error: "<<  sqrt(residuals.squaredNorm() / residuals.size()) << "\n";
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
                std::vector<Eigen::Vector3d> cams3, 
                std::vector<Eigen::Vector3d> ray_dirs3,
                std::vector<Eigen::Vector3d> world_pos){
    
    //std::cout << "lm driver has these cam centrums \n";
    //for(const auto& cam: cams){
    //    std::cout << cam << "\n \n";}


    MyFunctor functor(cams1,ray_dirs1, cams2,ray_dirs2,
                    cams3,ray_dirs3,  world_pos);
    Eigen::NumericalDiff<MyFunctor> numDiff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<MyFunctor>, double> solver(numDiff);


    // ai asetukset 
    solver.parameters.ftol     = 1e-12;    // Function tolerance
    solver.parameters.xtol     = 1e-12;    // Parameter tolerance
    solver.parameters.gtol     = 1e-8;     // Gradient tolerance (UNCOMMENT THIS)
    solver.parameters.epsfcn   = 1e-12;    // Finite diff precision
    solver.parameters.maxfev   = 10000;    // Max function evals
    solver.parameters.factor   = 1e-8;

    //solver.parameters.ftol = 1e-4;     
    //solver.parameters.xtol = 1e-6;      
    //solver.parameters.gtol = 1e-4;
    //solver.parameters.maxfev = 1000; 
    
    int total_cams = cams1.size() + cams2.size() + cams3.size();
    Eigen::Vector3d is = (cams1[0]+cams1[1]+ cams1[2]) / 3.0; // something to use as initial
    Eigen::VectorXd params(6 + total_cams);
    
    params << 0.0, 0.0, 0.0,  // Rotation vector (identity initially)
              is[0], is[1], is[2],  // Translation
              Eigen::VectorXd::Ones(total_cams);

    Eigen::AngleAxisd init_rot(90.0 * M_PI/180.0, Eigen::Vector3d(1,1,1).normalized()); // just something initially
    params.segment<3>(0) = init_rot.axis() * init_rot.angle();
    for(auto& ray : ray_dirs3) {
        std::cout << ray.norm();
    }


    int ret = solver.minimize(params);

    Eigen::VectorXd final_res;
    functor(params, final_res);
    std::cout << "Status: " << ret << ", RMSE: " << sqrt(final_res.squaredNorm()/final_res.size()) << "\n";


    //std::cout << params.segment<3>(3) <<"\n";
    //std::cout << " after solve" "\n";
    //std::cout <<" rots"  << params.segment<3>(0)   << "\n";
    //std::cout <<" trans"  << params.segment<3>(3)   << "\n";
    //std::cout <<" depths"  << params.segment(6, total_cams)  << "\n";

    return params;
    // https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm
}



#endif