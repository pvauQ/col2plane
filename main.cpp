
#include <iostream>
#include <string>
#include "src/col2plane.h"

int main(int argc, char* argv[])
{
    std::cout << "You have entered " << argc
         << " arguments:" << std::endl;

    std::string tag_txt = "";
    // Using a while loop to iterate through arguments
    int i = 0;
    while (i < argc) {
        std:: cout << "Argument " << i  << ": " << argv[i]
             << std:: endl;
        
        if (i ==  1){
            tag_txt = argv[i];
        }
        i++;
    }
    // ajetaanko cctag vai käytetäänkö tallennettuja.
    Col2Plane instance;
    if (tag_txt != ""){
        instance.withPrecalulated(tag_txt);
    }
    else{
        instance.calcCctag();
    }
    instance.col2CctSpace(4);

    return 0;
}