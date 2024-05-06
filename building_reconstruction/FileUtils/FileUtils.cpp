#include "FileUtils.h"

using namespace std;

void FileUtils::setInputFilePath(std::string file_path) {
    input_file_path = file_path;
}

std::string FileUtils::getInputFilePath() {
    return input_file_path;
}

vector<int> FileUtils::getLinesStartingWithString(std::string wordToSearch) {
    std::ifstream file(input_file_path);
    vector<int> indexes;
    if (file.is_open()) {
        std::string line;
        int index = 0;
        while (std::getline(file, line)) {
            if (line.substr(0, wordToSearch.size()) == wordToSearch) {
                indexes.push_back(index);
            }
            index++;
        }
        file.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }
    return indexes;
}

