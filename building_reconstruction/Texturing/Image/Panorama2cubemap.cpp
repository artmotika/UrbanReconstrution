#include "Panorama2cubemap.h"

using namespace std;
using namespace urban_rec;

void Panorama2cubemap::setImagePanorama(std::string image_path) {
    imagePanorama = cv::imread(image_path, cv::IMREAD_COLOR);
    input_width = imagePanorama.cols;
    input_height = imagePanorama.rows;
    float side_ratio = input_width / input_height;
    if (side_ratio != 2) {
        std::ostringstream oss;
        oss << "Input imagePanorama must have width 8000px and height 4000px,"
            << " but have width " << input_width << "and height " << input_height << "!\n";
        throw std::domain_error(oss.str());
    }
}

double Panorama2cubemap::degree_to_radian(double degree) {
    return degree * M_PI / 180.0;
}

void Panorama2cubemap::setInputImagePath(std::string image_path) {
    input_image_path = image_path;
}

std::string Panorama2cubemap::getInputImagePath() {
    return input_image_path;
}

void Panorama2cubemap::setOutputImagePath(std::string image_path) {
    output_image_path = image_path;
}

std::string Panorama2cubemap::getOutputImagePath() {
    return output_image_path;
}

void Panorama2cubemap::setDirPath(std::string path) {
    dir_path = path;
}

std::string Panorama2cubemap::getDirPath() {
    return dir_path;
}

void Panorama2cubemap::setDescriptionFileOption(bool description_file) {
    description_file_option = description_file;
}

bool Panorama2cubemap::getDescriptionFileOption() {
    return description_file_option;
}

void Panorama2cubemap::setFocalLengthW(double focal_length) {
    focal_length_w = focal_length;
}

double Panorama2cubemap::getFocalLengthW() {
    return focal_length_w;
}

void Panorama2cubemap::setFocalLengthH(double focal_length) {
    focal_length_h = focal_length;
}

double Panorama2cubemap::getFocalLengthH() {
    return focal_length_h;
}

void Panorama2cubemap::setFocalLength(double focal_length_width, double focal_length_height) {
    focal_length_w = focal_length_width;
    focal_length_h = focal_length_height;
}

double Panorama2cubemap::getFocalLength() {
    return focal_length_h;
}

void Panorama2cubemap::setShift(double x, double y, double z) {
    shift = make_tuple(x, y, z);
}

void Panorama2cubemap::setShift(shift_coord input_shift) {
    shift = input_shift;
}

shift_coord Panorama2cubemap::getShift() {
    return shift;
}

void Panorama2cubemap::setCsvFile(std::string file_path, int pColumnNameIdx, int pRowNameIdx, char separatorParams) {
    csv_file = rapidcsv::Document(file_path, rapidcsv::LabelParams(pColumnNameIdx, pRowNameIdx),
                                  rapidcsv::SeparatorParams(separatorParams));
}

rapidcsv::Document Panorama2cubemap::getCsvFile() {
    return csv_file;
}

void Panorama2cubemap::createDescriptionFiles(std::string output_image_subpath1) {
    std::ostringstream oss;
    oss << output_image_subpath1 << "Yplus" << ".txt";
    createDescriptionFile(DestinationType::Yplus, oss.str());
    oss.str("");
    oss << output_image_subpath1 << "Xplus" << ".txt";
    createDescriptionFile(DestinationType::Xplus, oss.str());
    oss.str("");
    oss << output_image_subpath1 << "Yminus" << ".txt";
    createDescriptionFile(DestinationType::Yminus, oss.str());
    oss.str("");
    oss << output_image_subpath1 << "Xminus" << ".txt";
    createDescriptionFile(DestinationType::Xminus, oss.str());
    oss.str("");
    oss << output_image_subpath1 << "Zminus" << ".txt";
    createDescriptionFile(DestinationType::Zminus, oss.str());
    oss.str("");
    oss << output_image_subpath1 << "Zplus" << ".txt";
    createDescriptionFile(DestinationType::Zplus, oss.str());
}

void Panorama2cubemap::createDescriptionFile(DestinationType type, std::string output_path) {
    double image_width = input_width / 4;
    double image_height = input_width / 4;
    double param;
    double radian;
    std::ofstream output_file(output_path);
    std::string image_name = path_utils::getFileName(input_image_path);
    double projectedX = csv_file.GetCell<double>("projectedX[m]", image_name) + std::get<0>(shift);
    double projectedY = csv_file.GetCell<double>("projectedY[m]", image_name) + std::get<1>(shift);
    double projectedZ = csv_file.GetCell<double>("projectedZ[m]", image_name) + std::get<2>(shift);
    double heading = csv_file.GetCell<double>("heading[deg]", image_name);
    double pitch = csv_file.GetCell<double>("pitch[deg]", image_name);
    double roll = csv_file.GetCell<double>("roll[deg]", image_name);
    output_file << projectedX << " " << projectedY << " " << projectedZ << std::endl;

    Eigen::Matrix3d m;
    Eigen::Quaterniond rollAngle;
    Eigen::Quaterniond headingAngle;
    Eigen::Quaterniond pitchAngle;

    double right_angle = degree_to_radian(90);

    switch (type) {
        case DestinationType::Yplus:
            param = right_angle;
            headingAngle = Eigen::AngleAxisd(2*right_angle - degree_to_radian(heading) + param, Eigen::Vector3d::UnitZ());
            pitchAngle = Eigen::AngleAxisd(degree_to_radian(pitch), Eigen::Vector3d::UnitY());
            rollAngle = Eigen::AngleAxisd(-degree_to_radian(roll) + 3*right_angle, Eigen::Vector3d::UnitX());
            cout << "Yplus; heading: " << heading << " roll: " << roll << " pitch: " << pitch << endl;
            break;
        case DestinationType::Xplus:
            headingAngle = Eigen::AngleAxisd(2*right_angle - degree_to_radian(heading), Eigen::Vector3d::UnitZ());
            pitchAngle = Eigen::AngleAxisd(-degree_to_radian(pitch), Eigen::Vector3d::UnitY());
            rollAngle = Eigen::AngleAxisd(-degree_to_radian(roll) + 3*right_angle, Eigen::Vector3d::UnitX());
            cout << "Xplus; heading: " << heading << " roll: " << roll << " pitch: " << pitch << endl;
            break;
        case DestinationType::Yminus:
            param = -right_angle;
            headingAngle = Eigen::AngleAxisd(2*right_angle - degree_to_radian(heading) + param, Eigen::Vector3d::UnitZ());
            pitchAngle = Eigen::AngleAxisd(-degree_to_radian(pitch), Eigen::Vector3d::UnitY());
            rollAngle = Eigen::AngleAxisd(degree_to_radian(roll) + 3*right_angle, Eigen::Vector3d::UnitX());
            cout << "Yminus; heading: " << heading << " roll: " << roll << " pitch: " << pitch << endl;
            break;
        case DestinationType::Xminus:
            param = 2*right_angle;
            headingAngle = Eigen::AngleAxisd(2*right_angle - degree_to_radian(heading) + param, Eigen::Vector3d::UnitZ());
            pitchAngle = Eigen::AngleAxisd(-degree_to_radian(pitch), Eigen::Vector3d::UnitY());
            rollAngle = Eigen::AngleAxisd(-degree_to_radian(roll) + 3*right_angle, Eigen::Vector3d::UnitX());
            cout << "Xminus; heading: " << heading << " roll: " << roll << " pitch: " << pitch << endl;
            break;
        case DestinationType::Zminus:
            headingAngle = Eigen::AngleAxisd(2*right_angle - degree_to_radian(heading), Eigen::Vector3d::UnitZ());
            pitchAngle = Eigen::AngleAxisd(degree_to_radian(pitch), Eigen::Vector3d::UnitY());
            rollAngle = Eigen::AngleAxisd(degree_to_radian(roll) + 2*right_angle, Eigen::Vector3d::UnitX());
            cout << "Zminus; heading: " << heading << " roll: " << roll << " pitch: " << pitch << endl;
            break;
        case DestinationType::Zplus:
            headingAngle = Eigen::AngleAxisd(2*right_angle - degree_to_radian(heading), Eigen::Vector3d::UnitZ());
            pitchAngle = Eigen::AngleAxisd(degree_to_radian(pitch), Eigen::Vector3d::UnitY());
            rollAngle = Eigen::AngleAxisd(-degree_to_radian(roll), Eigen::Vector3d::UnitX());
            cout << "Zplus; heading: " << heading << " roll: " << roll << " pitch: " << pitch << endl;
            break;
        default:
            output_file.close();
            std::ostringstream oss;
            oss << "DestinationType type" << type << " is incorrect!\n";
            throw invalid_argument(oss.str());
    }
    Eigen::Matrix3d rotation_matrix_x = rollAngle.toRotationMatrix();
    Eigen::Matrix3d rotation_matrix_y = pitchAngle.toRotationMatrix();
    Eigen::Matrix3d rotation_matrix_z = headingAngle.toRotationMatrix();
    m = rotation_matrix_z * rotation_matrix_y * rotation_matrix_x;
    output_file << m.coeff(0, 0) << " " << m.coeff(0, 1) << " " << m.coeff(0, 2) << std::endl;
    output_file << m.coeff(1, 0) << " " << m.coeff(1, 1) << " " << m.coeff(1, 2) << std::endl;
    output_file << m.coeff(2, 0) << " " << m.coeff(2, 1) << " " << m.coeff(2, 2) << std::endl;
    output_file << focal_length_w << std::endl;
    output_file << focal_length_h << std::endl;
    output_file << image_width << std::endl;
    output_file << image_height << std::endl;

    output_file.close();
}

void Panorama2cubemap::transform() {
    setImagePanorama(input_image_path);
    const int r = imagePanorama.cols / 4.0;
    const int output_width = r * 3;
    const int output_height = r * 2;

    // Placeholder image for the result
    vector <cv::Mat> destinations;
    for (int i = 0; i < 6; i++) {
        cv::Mat destination(r, r, CV_8UC3, cv::Scalar(255, 255, 255));
        destinations.push_back(destination);
    }

    auto begin = chrono::high_resolution_clock::now();

#pragma omp parallel for
    for (int j = 0; j < output_width; j++) {
        // #pragma omp parallel for
        for (int i = 0; i < output_height; i++) {
            DestinationType destImageType;
            float tx = 0.0;
            float ty = 0.0;
            float x = 0.0;
            float y = 0.0;
            float z = 0.0;

            if (i < r + 1) { // top half
                if (j < r + 1) { // +Y (left face)
                    destImageType = DestinationType::Yplus;
                    tx = j;
                    ty = i;
                    x = tx - 0.5 * r;
                    y = 0.5 * r;
                    z = ty - 0.5 * r;
                } else if (j < 2 * r + 1) { // +X (front face)
                    destImageType = DestinationType::Xplus;
                    tx = j - r;
                    ty = i;
                    x = 0.5 * r;
                    y = 0.5 * r - tx;
                    z = ty - 0.5 * r;
                } else { // -Y (right face)
                    destImageType = DestinationType::Yminus;
                    tx = j - r * 2;
                    ty = i;
                    x = 0.5 * r - tx;
                    y = -0.5 * r;
                    z = ty - 0.5 * r;
                }
            } else { // bottom half
                if (j < r + 1) { // -X (back face)
                    destImageType = DestinationType::Xminus;
                    tx = j;
                    ty = i - r;
                    x = -0.5 * r;
                    y = tx - 0.5 * r;
                    z = ty - 0.5 * r;
                } else if (j < 2 * r + 1) { // -Z (bottom face)
                    destImageType = DestinationType::Zminus;
                    tx = j - r;
                    ty = i - r;
                    x = 0.5 * r - ty;
                    y = 0.5 * r - tx;
                    z = 0.5 * r;
                } else { // +Z (top face)
                    destImageType = DestinationType::Zplus;
                    tx = j - r * 2;
                    ty = i - r;
                    x = ty - 0.5 * r;
                    y = 0.5 * r - tx;
                    z = -0.5 * r;
                }
            }

            // now find out the polar coordinates
            float rho = sqrt(x * x + y * y + z * z);
            float normTheta = (2*M_PI - atan2(y, x)) / (2*M_PI); // /(2*M_PI) normalise theta
            float normPhi = (M_PI - acos(z / rho)) / (M_PI); // /M_PI normalise phi

            // use this for coordinates
            float iX = normTheta * input_width;
            float iY = normPhi * input_height;

            destinations[destImageType].at<cv::Vec3b>(ty, tx) = imagePanorama.at<cv::Vec3b>(int(iY), int(iX));
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - begin;

    std::ostringstream oss;
    std::string output_image_subpath1;
    std::string output_image_subpath2;
    int dot_index = path_utils::getIndexBeforeChar(output_image_path, '.');
    output_image_subpath1 = output_image_path.substr(0, dot_index);
    output_image_subpath2 = output_image_path.substr(dot_index, output_image_path.size());

    oss << output_image_subpath1 << "Yplus" << output_image_subpath2;
    std::cout << oss.str() << std::endl;
    cv::imwrite(oss.str(), destinations[DestinationType::Yplus]);
    oss.str("");
    oss << output_image_subpath1 << "Xplus" << output_image_subpath2;
    std::cout << oss.str() << std::endl;
    cv::imwrite(oss.str(), destinations[DestinationType::Xplus]);
    oss.str("");
    oss << output_image_subpath1 << "Yminus" << output_image_subpath2;
    std::cout << oss.str() << std::endl;
    cv::imwrite(oss.str(), destinations[DestinationType::Yminus]);
    oss.str("");
    oss << output_image_subpath1 << "Xminus" << output_image_subpath2;
    std::cout << oss.str() << std::endl;
    cv::imwrite(oss.str(), destinations[DestinationType::Xminus]);
    oss.str("");
    oss << output_image_subpath1 << "Zminus" << output_image_subpath2;
    std::cout << oss.str() << std::endl;
    cv::imwrite(oss.str(), destinations[DestinationType::Zminus]);
    oss.str("");
    oss << output_image_subpath1 << "Zplus" << output_image_subpath2;
    std::cout << oss.str() << std::endl;
    cv::imwrite(oss.str(), destinations[DestinationType::Zplus]);

    if (description_file_option) createDescriptionFiles(output_image_subpath1);

    cout << "Processing time: " << diff.count() << " s" << endl;
}

void Panorama2cubemap::transform_dir(std::string file_extension) {
    const boost::filesystem::path base_dir(dir_path);
    std::string extension(file_extension);
    std::vector <boost::filesystem::path> filenames;
    try {
        for (boost::filesystem::directory_iterator it(base_dir);
             it != boost::filesystem::directory_iterator(); ++it) {
            if (boost::filesystem::is_regular_file(it->status()) &&
                boost::filesystem::extension(it->path()) == extension) {
                filenames.push_back(it->path());
            }
        }
    } catch (const boost::filesystem::filesystem_error &e) {
        cerr << e.what() << endl;
    }
    std::sort(filenames.begin(), filenames.end());

    for (int i = 0; i < filenames.size(); ++i) {
        string file_name = filenames[i].stem().string();
        if (!isdigit(file_name[file_name.length() - 1])) continue;
        input_image_path = filenames[i].string();

        int dot_index = path_utils::getIndexBeforeChar(input_image_path, '.');

        output_image_path = input_image_path.substr(0, dot_index) + "_cubemap.jpg";
        std::cout << "input_image_path: " << input_image_path << " output_image_path: " << output_image_path
                  << std::endl;

        imagePanorama.release();
        transform();
    }
}
