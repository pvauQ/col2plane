
#include <iostream>
#include <string>
#include "src/col2plane.h"

int main(int argc, char* argv[])
{

    std::map<std::string, std::string> param_value;

    if ( argc > 1 && std::string(argv[1]) == "--help"){
        std::cout <<
        " --solve_mode  p3p lm  . defaults to p3p \n"     
        //" --model_dir *  . n\n"
        " --cctag_from_file   .give a file path \n"
        " -- help \n";
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