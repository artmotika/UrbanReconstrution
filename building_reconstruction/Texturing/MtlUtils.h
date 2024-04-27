#ifndef URBAN_RECONSTRUCTION_MTLUTILS_H
#define URBAN_RECONSTRUCTION_MTLUTILS_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "../StringUtils/PathUtils.h"

class MtlUtils {
public:
    MtlUtils(std::string input_mtlfile_path) {
        setInputMtlfilePath(input_mtlfile_path);
    }

    void setInputMtlfilePath(std::string mtlfile_path);

    std::string getInputMtlfilePath();

    void addToMapKd(std::string extraString);

private:
    std::string input_mtlfile_path;

};


#endif //URBAN_RECONSTRUCTION_MTLUTILS_H
