#include "MtlUtils.h"
#include "../FileUtils/FileUtils.h"

using namespace std;

void MtlUtils::setInputMtlfilePath(std::string mtlfile_path) {
    input_mtlfile_path = mtlfile_path;
}

std::string MtlUtils::getInputMtlfilePath() {
    return input_mtlfile_path;
}

void MtlUtils::addToMapKd(std::string extraString) {
    FileUtils mtlFile = FileUtils(input_mtlfile_path);
    vector <int> change_line_indexes = mtlFile.getLinesStartingWithString("map_Kd");
    std::fstream file(input_mtlfile_path, std::ios::in | std::ios::out);

    if (file.is_open()) {
        vector <std::string> lines;
        std::string line;
        while (std::getline(file, line)) {
            lines.push_back(line);
        }
        file.clear();
        file.seekp(0, std::ios::beg);

        for (int indexToReplace : change_line_indexes) {
            std::string prevLine = lines[indexToReplace];
            std::ostringstream oss;
            std::string output_subpath1;
            std::string output_subpath2;
            int dot_index = path_utils::getIndexBeforeChar(prevLine, '.');
            output_subpath1 = prevLine.substr(0, dot_index);
            output_subpath2 = prevLine.substr(dot_index, prevLine.size());
            oss << output_subpath1 << extraString << output_subpath2;
            std::string newLine = oss.str(); // новая строка, которой нужно заменить старую
            lines[indexToReplace] = newLine;
        }

        for (const auto& l : lines) {
            file << l << std::endl;
        }
        file.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }
}