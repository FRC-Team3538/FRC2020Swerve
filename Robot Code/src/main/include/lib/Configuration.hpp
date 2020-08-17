#pragma once

#include "json.hpp"
#include <fstream>
#include <iostream>
#include <string>

class Configuration
{    
protected:    
    //TODO: This should differentiate between Comp, Practice, etc environments.
    // This can be done file-wise by adding Config files under subfolders, as seen in the example.
    // Checking for /home/lvuser/THIS_IS_THE_COMP_BOT or the alternative is how I've done it in the past.
    std::string get_path()  {
        return "/home/lvuser/config/";
    }
public:
    template<typename T>
    T Get(std::string file) {
        ifstream json_file(get_path() + file);
        if (json_file) {
            std::string contents;
            json_file.seekg(0, std::ios::end);
            contents.resize(json_file.tellg());
            json_file.seekg(0, std::ios::beg);
            json_file.read(&contents[0], contents.size());
            json_file.close();

            auto json_contents = nlohmann::json::parse(contents);
            return json_contents.get<T>();
        }
        
        throw errno;
    }
};