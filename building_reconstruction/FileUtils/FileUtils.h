#ifndef URBAN_RECONSTRUCTION_FILEUTILS_H
#define URBAN_RECONSTRUCTION_FILEUTILS_H

#include <iostream>
#include <fstream>
#include <string>

using namespace std;

class FileUtils {
public:
    FileUtils(std::string input_file_path) {
        setInputFilePath(input_file_path);
    }

    void setInputFilePath(std::string file_path);

    std::string getInputFilePath();

    vector <int> getLinesStartingWithString(std::string wordToSearch);

private:
    std::string input_file_path;
};

#endif //URBAN_RECONSTRUCTION_FILEUTILS_H
