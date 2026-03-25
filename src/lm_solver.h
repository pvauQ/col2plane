#ifndef USABLE_LM_SOLVER_H
#define USABLE_LM_SOLVER_H
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

class Functor_trans_lock {
private:
    std::vector<std::vector<Eigen::Vector3d>> cams;
    std::vector<std::vector<Eigen::Vector3d>> rays;
    std::vector<Eigen::Vector3d> X_targets;
    size_t n_cams_;
    
public:
    typedef double Scalar;
    enum { InputsAtCompileTime = Eigen::Dynamic, ValuesAtCompileTime = Eigen::Dynamic };
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    int inputs() const { return 6 + n_cams_; }
    int values() const { return 3 * n_cams_; }


    Functor_trans_lock(std::vector<std::vector<Eigen::Vector3d>> cams,
                std::vector<std::vector<Eigen::Vector3d>> rays,
                std::vector<Eigen::Vector3d> X_targets)
        :cams(cams),
        rays(rays),
        X_targets(X_targets){

            if (cams.size() != X_targets.size()) {std::cerr << "functor got more cam vectors than x_tagets broken reults \n"; }
                //TODO  throw?
            n_cams_ = 0;
            for (const auto& camlist : cams){
                n_cams_+= camlist.size();
            }
        }
    
    

    int operator()(const Eigen::VectorXd& params,  // from s ->w
                     Eigen::VectorXd& residuals) const {
        
        Eigen::Vector3d angle_ax = params.segment<3>(0);
        Eigen::Vector3d trans    = params.segment<3>(3);
        Eigen::VectorXd scales   = params.segment(6,n_cams_);

        Eigen::AngleAxisd R_aa(angle_ax.norm(), angle_ax.normalized());                
        Eigen::Matrix3d ROT = R_aa.toRotationMatrix();

        residuals.resize(3 * n_cams_);  // this is only needed when wrong size residuals are passed in ( probably manualy)
        
        int scale_ind =0;
        int res_ind = 0;
        size_t kissa =std::min(cams.size(),rays.size());

        // hack to test if optimization works better when not optimizing trans at all.
        //trans = Eigen::Vector3d(0.0, 0.0, 0.0);
        //ROT =  Eigen::Matrix3d::Identity();

        // all cameras that see first marker
        // all cameras that see second marker...
        // all have 3 residuals virhe targettiin nähden xyz
        for (size_t i = 0; i<kissa; i++){
            //cams that see ith marker
            const std::vector<Eigen::Vector3d>& cam_group = cams[i];
            const std::vector<Eigen::Vector3d>& ray_group = rays[i];
            assert(cam_group.size() == ray_group.size());
            
            size_t koira = std::min(cam_group.size(), ray_group.size());
            for(size_t j = 0; j<koira; j++){
                const Eigen::Vector3d cam = cam_group[j];
                const Eigen::Vector3d ray = ray_group[j];
                Eigen::Vector3d pred = ROT* cam + trans + scales[scale_ind]* ROT * ray;
                scale_ind++;
                residuals.segment<3>(res_ind) = pred - X_targets[i];
                res_ind +=3; // we have 3 residuals  per cam.

            }
        }
        return 0;
    }
};


/// LOOCKEED RAAAYS----------------------------------------------------
class Functor_ray_lock {
private:
    std::vector<std::vector<Eigen::Vector3d>> cams;
    std::vector<std::vector<Eigen::Vector3d>> rays;
    std::vector<Eigen::Vector3d> X_targets;
    Eigen::VectorXd scales;
    size_t n_cams_;
    
public:
    typedef double Scalar;
    enum { InputsAtCompileTime = Eigen::Dynamic, ValuesAtCompileTime = Eigen::Dynamic };
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    int inputs() const { return 6 ; }
    int values() const { return 3 * n_cams_; }


    Functor_ray_lock(std::vector<std::vector<Eigen::Vector3d>> cams,
                std::vector<std::vector<Eigen::Vector3d>> rays,
                std::vector<Eigen::Vector3d> X_targets,
                Eigen::VectorXd scales)
        :cams(cams),
        rays(rays),
        scales(scales),
        X_targets(X_targets){

            if (cams.size() != X_targets.size()) {std::cerr << "functor got more cam vectors than x_tagets broken reults \n"; }
                //TODO  throw?
            n_cams_ = 0;
            for (const auto& camlist : cams){
                n_cams_+= camlist.size();
            }
        }
    
    

    int operator()(const Eigen::VectorXd& params,  // from s ->w
                     Eigen::VectorXd& residuals) const {
        
        Eigen::Vector3d angle_ax = params.segment<3>(0);
        Eigen::Vector3d trans    = params.segment<3>(3);
        

        Eigen::AngleAxisd R_aa(angle_ax.norm(), angle_ax.normalized());                
        Eigen::Matrix3d ROT = R_aa.toRotationMatrix();

        residuals.resize(3 * n_cams_);  // this is only needed when wrong size residuals are passed in ( probably manualy)
        
        int scale_ind =0;
        int res_ind = 0;
        size_t kissa =std::min(cams.size(),rays.size());


        // all cameras that see first marker
        // all cameras that see second marker...
        // all have 3 residuals virhe targettiin nähden xyz
        for (size_t i = 0; i<kissa; i++){
            //cams that see ith marker
            const std::vector<Eigen::Vector3d>& cam_group = cams[i];
            const std::vector<Eigen::Vector3d>& ray_group = rays[i];
            assert(cam_group.size() == ray_group.size());
            
            size_t koira = std::min(cam_group.size(), ray_group.size());
            for(size_t j = 0; j<koira; j++){
                const Eigen::Vector3d cam = cam_group[j];
                const Eigen::Vector3d ray = ray_group[j];
                Eigen::Vector3d pred = ROT* cam + trans + scales[scale_ind]* ROT * ray;
                scale_ind++;
                residuals.segment<3>(res_ind) = pred - X_targets[i];
                res_ind +=3; // we have 3 residuals  per cam.

            }
        }
        return 0;
    }
};



Eigen::VectorXd lmDriver(std::vector<std::vector<Eigen::Vector3d>> cams,
                    std::vector<std::vector<Eigen::Vector3d>> rays,
                        std::vector<Eigen::Vector3d> world_pos){
    



    //std::cout << "lm driver has these cam centrums \n";
    //for(const auto& cam: cams){
    //    std::cout << cam << "\n \n";}


    Functor_trans_lock functor(cams, rays, world_pos);
    Eigen::NumericalDiff<Functor_trans_lock> numDiff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<Functor_trans_lock>, double> solver(numDiff);


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
    
    int total_cams =0;
    for(const auto& cam_group : cams){
        total_cams += cam_group.size();
    }
    Eigen::Vector3d is = cams[0][0]; // something to use as initial
    Eigen::VectorXd params(6 + total_cams);
    
    params << 0.0, 0.0, 0.0,  // Rotation vector (identity initially)
            is[0], is[1], is[2],  // Translation
            Eigen::VectorXd::Ones(total_cams);
    //Eigen::AngleAxisd init_rot(90.0 * M_PI/180.0, Eigen::Vector3d(1,1,1).normalized()); // just something initially
    //params.segment<3>(0) = init_rot.axis() * init_rot.angle();


    int ret = solver.minimize(params);

    Eigen::VectorXd final_res;
    functor(params, final_res);
    std::cout << "\nLM solver RMSE: " << sqrt(final_res.squaredNorm()/final_res.size()) << "\n";



    // pass first solution into second solver-------------------------- where we dont refine rays anymore
    
    /*
    Eigen::VectorXd scales = params.segment(6,total_cams);
    Functor_ray_lock functor2(cams, rays, world_pos, scales);
    Eigen::NumericalDiff<Functor_ray_lock> numDiff2(functor2);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<Functor_ray_lock>, double> solver2(numDiff2);

    Eigen::VectorXd params2(6);
    //params2 = params.segment(0,6);
    params2 << 0.0, 0.0, 0.0,  // Rotation vector (identity initially)
            is[0]+1, is[1]-1, is[2]+0.5;
    //std::cout << params2   << "\n";
    solver2.minimize(params2);
    
    Eigen::VectorXd final_res2;
    functor2(params2, final_res2);
    //std::cout << final_res2 << "\n";
    std::cout << "\n second LM solver RMSE: " << sqrt(final_res2.squaredNorm()/final_res2.size()) << "\n";


    //std::cout << params.segment<3>(3) <<"\n";
    //std::cout << " after solve" "\n";
    //std::cout <<" rots"  << params.segment<3>(0)   << "\n";
    //std::cout <<" trans"  << params.segment<3>(3)   << "\n";
    //std::cout <<" depths"  << params.segment(6, total_cams)  << "\n";
    
    //return params; u  need to return this if u want ray depths.. 
    Eigen::VectorXd params_to_ret(6 + total_cams);
    params_to_ret << params2 , params.segment(6, total_cams);

    return params_to_ret;
    */
   return params;
}

#endif