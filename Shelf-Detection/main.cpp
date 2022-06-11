#include <fstream>

#include "include/shelf-detection.hpp"
#include "include/settings_loader.hpp"

int main(int argc, char **argv) {
    if(argc != 3) {
        std::cout << "Expected two (2) arguments: ./shelf-detection <path to map> <path to settings>\n";
        return -1; 
    }
    std::string filename(argv[1]);
    std::string settings(argv[2]);

    auto params = getParameters(settings);
    ShelfDetection detector(filename, params);
    return 0;
}