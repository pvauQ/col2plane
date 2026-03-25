
#include <iostream>
#include <string>
#include "src/col2plane.h"

int main(int argc, char* argv[])
{

    std::map<std::string, std::string> param_value;

    if ( argc > 1 && std::string(argv[1]) == "--help"){
        std::cout <<
        "all files from user should be in  /photodir/malli \n"
        " --solve_mode  p3p lm  . defaults to lm \n"     
        //" --model_dir *  . n\n"
        " --cctag_from_file   give a file name  (in prementioned folder) \n"
        " --images_to_use   max images per tag to use. \n" 
        "--max_rep_error  max reproj error of img/camera.(colmap frame) " 
        " -- help  \n";
        return 0;
    }
    for(int i = 1; i < argc - 1; i += 2){
        std::string param,value;
        param = argv[i];
        value = argv[i+1];
        param_value.emplace(param,value);    
    }

    Col2Plane instance;

    // cctag_from_file
    auto it = param_value.find("--cctag_from_file");
    if (it != param_value.end()) {
        instance.withPrecalulated(it->second);
        it->second;
    }
    else{
        instance.calcCctag();
    }
    
    
    it = param_value.find("--images_to_use");
        if (it != param_value.end()) {
            int images_to_use = std::stoi(it->second);
            instance.number_of_images_lm = images_to_use;
        }

        it = param_value.find("--max_rep_error");
        if (it != param_value.end()) {
            double max_error = std::stod(it->second);
            instance.max_rep_error = max_error;
        }

    solve_mode mode(solve_mode::LM_SOLVE);
    it = param_value.find("--solve_mode");
        if (it != param_value.end()) {
            if (it->second == "p3p")
                mode = solve_mode::P3P_SOLVE;
            else if (it->second == "lm")
                mode = solve_mode::LM_SOLVE;
            else if (it->second == "ellipse"){
                std::cout << "not implemented";
                return 0;
            }
            
        }
    instance.col2CctSpace(mode);

    return 0;
}