#pragma once

#include <string>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>

class File {
public:
    File() = delete;
    File(std::string inFile) {
        parseFilename(inFile);
    }

    bool isEmpty() { 
        if(!fileEnding.size() || !filename.size()) { return true; }

        return false;
    }
    bool isPLY() { return fileEnding == "ply"; }
    bool isPCD() { return fileEnding == "pcd"; }
    std::string name() { return filename + "." + fileEnding; }


private:
    void generateName(std::vector<std::string>& splitNames) {
        for(size_t i = 0; i < splitNames.size() - 1; i++) {
            filename += splitNames[i];
            if(splitNames.size() > 2 && i < splitNames.size() - 2)
                filename += ".";
        }
    }
    void parseFilename(std::string filename) {
        std::vector<std::string> splitFilename;
        std::stringstream ss(filename);
        std::string token;
        
        while(std::getline(ss, token, '.')) {
            splitFilename.push_back(token);
        }

        if(splitFilename.size() >= 2) {
            generateName(splitFilename);
            fileEnding = splitFilename[splitFilename.size() - 1];
        }
    }    
    
    std::string fileEnding;
    std::string filename;
};