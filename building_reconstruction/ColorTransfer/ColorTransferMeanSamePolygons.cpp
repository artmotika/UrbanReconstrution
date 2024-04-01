#include "ColorTransferMeanSamePolygons.h"

using namespace cv;
using namespace std;
using namespace pcl;
using namespace Geometry_pcl;

void ColorTransferMeanSamePolygons::setDirPath(std::string image_path) {
    dir_path = image_path;
}

string ColorTransferMeanSamePolygons::getDirPath() {
    return dir_path;
}

void ColorTransferMeanSamePolygons::setNumberCams(int num) {
    number_cams = num;
}

int ColorTransferMeanSamePolygons::getNumberCams() {
    return number_cams;
}

void ColorTransferMeanSamePolygons::setInputPolygonMesh(pcl::PolygonMesh &mesh) {
    triangles = mesh;
}

void ColorTransferMeanSamePolygons::setInputTextureMesh(pcl::TextureMesh &mesh) {
    main_mesh = mesh;
}

void ColorTransferMeanSamePolygons::setInputTextureMeshes(vector<pcl::TextureMesh> &meshes) {
    input_meshes = meshes;
}

vector <vector <bool>> ColorTransferMeanSamePolygons::getBorderTp() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2(main_mesh.cloud, *input_cloud);
    // Для каждой камеры хранит массив, где по индексу вершины определяется индексы полигонов,
    // которые содержат данную вершину
    vector < vector <pcl::Indices>> mesh_points_idx_polygons_idx(number_cams, vector<pcl::Indices>(input_cloud->size()));
    // Заполняем индексы точек полигонов для каждой камеры
    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        for (int face_idx = 0; face_idx < main_mesh.tex_polygons[current_cam].size(); face_idx++) {
            pcl::Vertices polygon = main_mesh.tex_polygons[current_cam][face_idx];
            // Соответствие индексу вершины индексу грани в текстурированном меше
            mesh_points_idx_polygons_idx[current_cam][polygon.vertices[0]].push_back(face_idx);
            mesh_points_idx_polygons_idx[current_cam][polygon.vertices[1]].push_back(face_idx);
            mesh_points_idx_polygons_idx[current_cam][polygon.vertices[2]].push_back(face_idx);
        }
    }

    std::ofstream output_file("../example_mini5/log/log5.txt");
    // Для кажой камеры храним для кадого индекса грани, является ли он границей патча
    vector< vector <bool>> border_cams_to_face_idx(number_cams);

    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        vector <bool> faceToIsBorder(main_mesh.tex_polygons[current_cam].size(), false);
        border_cams_to_face_idx[current_cam] = faceToIsBorder;
    }

    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        // Проходим по каждой точке в облаке для current_cam
        for (int j = 0; j < main_mesh.tex_polygons[current_cam].size(); j++) {
            // Найти центр по координатам
            pcl::Vertices polygon = main_mesh.tex_polygons[current_cam][j];
            // Находим точку среди облаков других камер
            for (int cam_idx = current_cam+1; cam_idx < number_cams; cam_idx++) {
                for (int vertex_idx : polygon.vertices) {
                    for (int face_idx : mesh_points_idx_polygons_idx[cam_idx][vertex_idx]) {
                        border_cams_to_face_idx[current_cam][j] = true;
                        border_cams_to_face_idx[cam_idx][face_idx] = true;
                    }
                }
            }
        }
    }

    output_file.close();
    return border_cams_to_face_idx;
}

tuple< vector <int>, vector <int>> ColorTransferMeanSamePolygons::getFaceIndexMaps(vector <pcl::Vertices> &polygons,
                                                            vector <pcl::Vertices> &tex_polygons,
                                                            int num_points,
                                                            std::ofstream &output_file) {
    vector <int> faceIndexMapFullToPart(polygons.size(), -1);
    vector <int> faceIndexMapPartToFull(tex_polygons.size(), -1);
    vector <set <int>> pointIndex1ToFaceIndex(num_points);
    vector <set <int>> pointIndex2ToFaceIndex(num_points);
    vector <set <int>> pointIndex3ToFaceIndex(num_points);

    for (int face_idx = 0; face_idx < tex_polygons.size(); face_idx++) {
        pcl::Vertices polygon = tex_polygons[face_idx];
        pointIndex1ToFaceIndex[polygon.vertices[0]].insert(face_idx);
        pointIndex2ToFaceIndex[polygon.vertices[1]].insert(face_idx);
        pointIndex3ToFaceIndex[polygon.vertices[2]].insert(face_idx);
    }

//    // ONLY LOG
//    for (int face_idx = 0; face_idx < tex_polygons.size(); face_idx++) {
//        pcl::Vertices polygon = tex_polygons[face_idx];
//        output_file << "index0: " << polygon.vertices[0] << endl;
//        for (int n : pointIndex1ToFaceIndex[polygon.vertices[0]]) {
//            output_file << n << " ";
//        }
//        output_file << endl;
//        output_file << "index1: " << polygon.vertices[1] << endl;
//        for (int n : pointIndex2ToFaceIndex[polygon.vertices[1]]) {
//            output_file << n << " ";
//        }
//        output_file << endl;
//        output_file << "index2: " << polygon.vertices[2] << endl;
//        for (int n : pointIndex3ToFaceIndex[polygon.vertices[2]]) {
//            output_file << n << " ";
//        }
//        output_file << endl << endl;
//    }
//    // ONLY LOG

    for (int i = 0; i < polygons.size(); i++) {
        pcl::Vertices polygon = polygons[i];
        set <int> set1 = pointIndex1ToFaceIndex[polygon.vertices[0]];
        set <int> set2 = pointIndex2ToFaceIndex[polygon.vertices[1]];
        set <int> set3 = pointIndex3ToFaceIndex[polygon.vertices[2]];
        std::set<int> intersection;
        // Находим пересечение между первыми двумя множествами
        std::set_intersection(set1.begin(), set1.end(), set2.begin(), set2.end(),
                              std::inserter(intersection, intersection.begin()));
        // Находим пересечение между результатом и третьим множеством
        std::set<int> result_intersection;
        std::set_intersection(intersection.begin(), intersection.end(), set3.begin(), set3.end(),
                              std::inserter(result_intersection, result_intersection.begin()));

        if (!result_intersection.empty()) {
            faceIndexMapFullToPart[i] = *(result_intersection.begin());
            faceIndexMapPartToFull[*(result_intersection.begin())] = i;
        }
    }
    return make_tuple(faceIndexMapFullToPart, faceIndexMapPartToFull);
}

void ColorTransferMeanSamePolygons::transferColorBetweenPolygons(int cur_cam,
                                                          vector <PolygonTextureCoords> &camToPolygonTextureCoords,
                                                          vector <vector <int>> &meshFaceIndexMapInputMeshesFullToPart,
                                                          vector <vector <int>> &meshFaceIndexMapInputMeshesPartToFull,
                                                          vector <cv::Mat> &textures,
                                                          cv::Mat &dest_target, std::ofstream & output_file,
                                                          vector <cv::Mat> &destinations) {
    double max_area = 0.0;
    int max_area_cam_idx = -1;
    for (int current_cam = 0; current_cam < number_cams ; current_cam++) { //&& current_cam != cur_cam
        PolygonTextureCoords polygonTextureCoords = camToPolygonTextureCoords[current_cam];
        double area = Geometry_pcl::triangle_area(polygonTextureCoords.a, polygonTextureCoords.b, polygonTextureCoords.c);
//        output_file << "area: " << area << endl;
//        output_file << "a: " << polygonTextureCoords.a.x << " "  << polygonTextureCoords.a.y
//        << " b: "<< polygonTextureCoords.b.x << " " << polygonTextureCoords.b.y
//        << " c: " << polygonTextureCoords.c.x << " " << polygonTextureCoords.c.y << endl;
        if (max_area < area) {
            max_area = area;
            max_area_cam_idx = current_cam;
        }
    }
    if (max_area_cam_idx == -1) return;
    output_file << "after continue..." << endl;

    PolygonTextureCoords polygonTextureCoordsSrc = camToPolygonTextureCoords[max_area_cam_idx];
    PolygonTextureCoords polygonTextureCoordsTarget = camToPolygonTextureCoords[cur_cam];
    int t1_x0 = int(polygonTextureCoordsSrc.a.x * texture_width);
    int t1_y0 = int(texture_height - polygonTextureCoordsSrc.a.y * texture_height);
    int t1_x1 = int(polygonTextureCoordsSrc.b.x * texture_width);
    int t1_y1 = int(texture_height - polygonTextureCoordsSrc.b.y * texture_height);
    int t1_x2 = int(polygonTextureCoordsSrc.c.x * texture_width);
    int t1_y2 = int(texture_height - polygonTextureCoordsSrc.c.y * texture_height);

    int t2_x0 = int(polygonTextureCoordsTarget.a.x * texture_width);
    int t2_y0 = int(texture_height - polygonTextureCoordsTarget.a.y * texture_height);
    int t2_x1 = int(polygonTextureCoordsTarget.b.x * texture_width);
    int t2_y1 = int(texture_height - polygonTextureCoordsTarget.b.y * texture_height);
    int t2_x2 = int(polygonTextureCoordsTarget.c.x * texture_width);
    int t2_y2 = int(texture_height - polygonTextureCoordsTarget.c.y * texture_height);

    int t1_maxX = std::max( t1_x0, std::max( t1_x1, t1_x2) );
    int t1_maxY = std::max( t1_y0, std::max( t1_y1, t1_y2) );
    int t1_minX = std::min( t1_x0, std::min( t1_x1, t1_x2) );
    int t1_minY = std::min( t1_y0, std::min( t1_y1, t1_y2) );

    int t2_maxX = std::max( t2_x0, std::max( t2_x1, t2_x2) );
    int t2_maxY = std::max( t2_y0, std::max( t2_y1, t2_y2) );
    int t2_minX = std::min( t2_x0, std::min( t2_x1, t2_x2) );
    int t2_minY = std::min( t2_y0, std::min( t2_y1, t2_y2) );

//    // Получаем средние значения цветов для исходного и целевого изображения
//    int blueSumSrc = 0;
//    int greenSumSrc = 0;
//    int redSumSrc = 0;
//    int countSrc = 0;
//
    for (int x = t1_minX; x <= t1_maxX; x++) {
        for (int y = t1_minY; y <= t1_maxY; y++) {
            // Берем MeanColor не из треугольника, а из прямоугольника, который содержит этот треугольник
            // (чтобы учесть случай неправильной позиции камеры при дальнем расстоянии)
            pcl::PointXY pt(float(x)/float(texture_width), float(texture_height - y)/float(texture_height));
            if (checkPointInsideTriangle(polygonTextureCoordsSrc.a, polygonTextureCoordsSrc.b, polygonTextureCoordsSrc.c, pt)) {
//                cv::Vec3b pixel = textures[max_area_cam_idx].at<cv::Vec3b>(y, x);
//                blueSumSrc += int(pixel[0]); // синий
//                greenSumSrc += int(pixel[1]); // зеленый
//                redSumSrc += int(pixel[2]); // красный
//                countSrc++;
                destinations[max_area_cam_idx].at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
            }
        }
    }
//
//    int blueSumTarget = 0;
//    int greenSumTarget = 0;
//    int redSumTarget = 0;
//    int countTarget = 0;
//
//    for (int x = t2_minX; x <= t2_maxX; x++) {
//        for (int y = t2_minY; y <= t2_maxY; y++) {
//            pcl::PointXY pt(float(x)/float(texture_width), float(texture_height - y)/float(texture_height));
//            if (checkPointInsideTriangle(polygonTextureCoordsTarget.a, polygonTextureCoordsTarget.b, polygonTextureCoordsTarget.c, pt)) {
//                cv::Vec3b pixel = textures[cur_cam].at<cv::Vec3b>(y, x);
//                blueSumTarget += int(pixel[0]); // синий
//                greenSumTarget += int(pixel[1]); // зеленый
//                redSumTarget += int(pixel[2]); // красный
//                countTarget++;
//            }
//        }
//    }
//
//    if (countTarget == 0 || countSrc == 0
//        || blueSumSrc + greenSumSrc + redSumSrc == 0
//        || blueSumTarget + greenSumTarget + redSumTarget == 0) {
//        return;
//    }
//    // Рассчитываем коэффициенты для смешивания цветов
//    double alpha;
//    double beta;
//    double gamma;
//    double meanBlueSrc = double(blueSumSrc) / double(countSrc);
//    double meanBlueTarget = double(blueSumTarget) / double(countTarget);
//    double meanGreenSrc = double(greenSumSrc) / double(countSrc);
//    double meanGreenTarget = double(greenSumTarget) / double(countTarget);
//    double meanRedSrc = double(redSumSrc) / double(countSrc);
//    double meanRedTarget = double(redSumTarget) / double(countTarget);
//    if (blueSumSrc == 0) {
//        alpha = 1.0;
//    } else {
//        alpha = meanBlueSrc / meanBlueTarget;
//    }
//    if (greenSumSrc == 0) {
//        beta = 1.0;
//    } else {
//        beta = meanGreenSrc / meanGreenTarget;
//    }
//    if (redSumSrc == 0) {
//        gamma = 1.0;
//    } else {
//        gamma = meanRedSrc / meanRedTarget;
//    }

    for (int x = t2_minX; x <= t2_maxX; x++) {
        for (int y = t2_minY; y <= t2_maxY; y++) {
            pcl::PointXY pt(float(x)/float(texture_width), float(texture_height - y)/float(texture_height));
            if (checkPointInsideTriangle(polygonTextureCoordsTarget.a, polygonTextureCoordsTarget.b, polygonTextureCoordsTarget.c, pt)) {
//                cv::Vec3b pixel = dest_target.at<cv::Vec3b>(y, x);
//                output_file << "pixel[0]: " << double(pixel[0]) << " pixel[1]: " << double(pixel[1])
//                            << " pixel[2]: " << double(pixel[2]) << endl
//                            << " alpha: " << alpha
//                            << " beta: " << beta
//                            << " gamma: " << gamma << endl;
//
//                pixel[0] = saturate_cast<uchar>(double(pixel[0]) * alpha);
//                pixel[1] = saturate_cast<uchar>(double(pixel[1]) * beta);
//                pixel[2] = saturate_cast<uchar>(double(pixel[2]) * gamma);
//                output_file << "pixel[0]: " << double(pixel[0]) << " pixel[1]: " << double(pixel[1])
//                            << " pixel[2]: " << double(pixel[2]) << endl << endl << endl;
////                dest_target.at<cv::Vec3b>(y, x) = pixel;
//                dest_target.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
            }
        }
    }
}

void ColorTransferMeanSamePolygons::transferColorBetweenTpBorder(vector< vector <bool>> &border_cams_to_face_idx,
                                                          vector <vector <int>> &meshFaceIndexMapMainMeshPartToFull,
                                                          vector <vector <int>> &meshFaceIndexMapInputMeshesFullToPart,
                                                          vector <vector <int>> &meshFaceIndexMapInputMeshesPartToFull,
                                                          vector <cv::Mat> &textures,
                                                          vector <cv::Mat> &destinations) {
    std::ofstream output_file("../example_mini5/log/log3.txt");
    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        cv::Mat dest_target = destinations[current_cam];
        for (int face_idx = 0; face_idx < main_mesh.tex_polygons[current_cam].size(); face_idx++) {
//            if (border_cams_to_face_idx[current_cam][face_idx]) {
                int global_target_face_idx = meshFaceIndexMapMainMeshPartToFull[current_cam][face_idx];
//                int target_face_idx = meshFaceIndexMapInputMeshesFullToPart[current_cam][global_target_face_idx];
                int target_face_idx = face_idx;
                output_file << "current_cam: " << current_cam << " face_idx: " << face_idx
                << " global_target_face_idx: " << global_target_face_idx
                << " target_face_idx: " << target_face_idx << endl;
                vector <PolygonTextureCoords> camToPolygonTextureCoords(number_cams);
                // copy UV coordinates for current_cam of face with face_idx
                pcl::PointXY p0 = PointXY(input_meshes[current_cam].tex_coordinates[0][target_face_idx * 3](0),
                                          input_meshes[current_cam].tex_coordinates[0][target_face_idx * 3](1));
                pcl::PointXY p1 = PointXY(input_meshes[current_cam].tex_coordinates[0][target_face_idx * 3 + 1](0),
                                          input_meshes[current_cam].tex_coordinates[0][target_face_idx * 3 + 1](1));
                pcl::PointXY p2 = PointXY(input_meshes[current_cam].tex_coordinates[0][target_face_idx * 3 + 2](0),
                                          input_meshes[current_cam].tex_coordinates[0][target_face_idx * 3 + 2](1));
//                output_file << "a: " << p0.x << " "  << p0.y
//                            << " b: "<< p1.x << " " << p1.y
//                            << " c: " << p2.x << " " << p2.y << endl;
                PolygonTextureCoords polygonTextureCoords = {p0, p1, p2};
                camToPolygonTextureCoords[current_cam] = polygonTextureCoords;
                for (int cam_idx = 0; cam_idx < number_cams ; cam_idx++) { //&& cam_idx != current_cam
//                    int src_face_idx = meshFaceIndexMapInputMeshesFullToPart[cam_idx][global_target_face_idx];
                    int src_face_idx = meshFaceIndexMapMainMeshFullToPart[cam_idx][global_target_face_idx];
                    if (src_face_idx != -1) {
                        cout << "src_face_idx: " << src_face_idx << endl;
                        // copy UV coordinates for current_cam of face with j
                        pcl::PointXY p0 = PointXY(input_meshes[cam_idx].tex_coordinates[0][src_face_idx * 3](0),
                                                  input_meshes[cam_idx].tex_coordinates[0][src_face_idx * 3](1));
                        pcl::PointXY p1 = PointXY(input_meshes[cam_idx].tex_coordinates[0][src_face_idx * 3 + 1](0),
                                                  input_meshes[cam_idx].tex_coordinates[0][src_face_idx * 3 + 1](1));
                        pcl::PointXY p2 = PointXY(input_meshes[cam_idx].tex_coordinates[0][src_face_idx * 3 + 2](0),
                                                  input_meshes[cam_idx].tex_coordinates[0][src_face_idx * 3 + 2](1));
                        output_file << "a: " << p0.x << " "  << p0.y
                                    << " b: "<< p1.x << " " << p1.y
                                    << " c: " << p2.x << " " << p2.y << endl << endl;
                        PolygonTextureCoords polygonTextureCoords = {p0, p1, p2};
                        camToPolygonTextureCoords[cam_idx] = polygonTextureCoords;
                    }
                }
                transferColorBetweenPolygons(current_cam,
                                             camToPolygonTextureCoords,
                                             meshFaceIndexMapInputMeshesFullToPart,
                                             meshFaceIndexMapInputMeshesPartToFull,
                                             textures, dest_target, output_file, destinations);
//            }
        }
    }
    output_file.close();
}

void ColorTransferMeanSamePolygons::transfer() {
    std::ofstream output_file("../example_mini5/log/log2.txt");
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2(triangles.cloud, *input_cloud);
    vector <vector <bool>> border_cams_to_face_idx = getBorderTp();
    vector <vector <int>> meshFaceIndexMapInputMeshesFullToPart(number_cams);
    vector <vector <int>> meshFaceIndexMapMainMeshFullToPart(number_cams);
    vector <vector <int>> meshFaceIndexMapInputMeshesPartToFull(number_cams);
    vector <vector <int>> meshFaceIndexMapMainMeshPartToFull(number_cams);

    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        output_file << "current_cam: " << current_cam
        << "polygons size = " << input_meshes[current_cam].tex_polygons[0].size() << endl;
        tuple< vector <int>, vector <int>> faceIndexMaps = getFaceIndexMaps(triangles.polygons,
                                                                            input_meshes[current_cam].tex_polygons[0],
                                                                            input_cloud->size(), output_file);
        meshFaceIndexMapInputMeshesFullToPart[current_cam] = get<0>(faceIndexMaps);
        meshFaceIndexMapInputMeshesPartToFull[current_cam] = get<1>(faceIndexMaps);
                output_file << "current_cam MAINMESH: " << current_cam
        << "polygons size = " << main_mesh.tex_polygons[current_cam].size() << endl;
        faceIndexMaps = getFaceIndexMaps(triangles.polygons,
                                        main_mesh.tex_polygons[current_cam],
                                        input_cloud->size(), output_file);
        meshFaceIndexMapMainMeshFullToPart[current_cam] = get<0>(faceIndexMaps);
        meshFaceIndexMapMainMeshPartToFull[current_cam] = get<1>(faceIndexMaps);
    }

    //LOG ONLY
    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        output_file << "current_cam: " << current_cam << endl;
        vector <int> v1 = meshFaceIndexMapInputMeshesFullToPart[current_cam];
        vector <int> v2 = meshFaceIndexMapInputMeshesPartToFull[current_cam];
        vector <int> v3 = meshFaceIndexMapMainMeshFullToPart[current_cam];
        vector <int> v4 = meshFaceIndexMapMainMeshPartToFull[current_cam];
        for (int i = 0; i < v1.size(); i++) {
            output_file << "meshFaceIndexMapInputMeshesFullToPart " << "full: " << i << " part: " << v1[i] << endl;
        }
        for (int i = 0; i < v2.size(); i++) {
            output_file << "meshFaceIndexMapInputMeshesPartToFull " << "part: " << i << " full: " << v2[i] << endl;
        }
        for (int i = 0; i < v3.size(); i++) {
            output_file << "meshFaceIndexMapMainMeshFullToPart " << "full: " << i << " part: " << v3[i] << endl;
        }
        for (int i = 0; i < v4.size(); i++) {
            output_file << "meshFaceIndexMapMainMeshPartToFull " << "part: " << i << " full: " << v4[i] << endl;
        }
        output_file << "\n\n\n";
    }
    //LOG ONLY

    vector <cv::Mat> textures;
    vector <cv::Mat> destinations;
    vector <string> dest_paths;
    // Сохраняем текстуры изображения в матрицы из opencv
    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        // Создание отдельных файлов-текстур
        std::ostringstream oss_in;
        std::ostringstream oss_dest;

        oss_in << dir_path << main_mesh.tex_materials[current_cam].tex_file;
        string image_path = oss_in.str();

        int dot_index = path_utils::getIndexBeforeChar(image_path, '.');
        string image_subpath1 = image_path.substr(0, dot_index);
        string image_subpath2 = image_path.substr(dot_index, image_path.size());

        oss_dest << image_subpath1 << "_Tpatch" << image_subpath2;
        string dest_path = oss_dest.str();

        cout << image_path << endl;
        cout << dest_path << endl;
        dest_paths.push_back(dest_path);

        cv::Mat imagePatch = cv::imread(image_path, cv::IMREAD_COLOR);
        texture_height = imagePatch.rows;
        texture_width = imagePatch.cols;
        cv::Mat destPatch = imagePatch.clone();
        textures.push_back(imagePatch);
        destinations.push_back(destPatch);
    }

    transferColorBetweenTpBorder(border_cams_to_face_idx,
                                 meshFaceIndexMapMainMeshPartToFull,
                                 meshFaceIndexMapInputMeshesFullToPart,
                                 meshFaceIndexMapInputMeshesPartToFull,
                                 textures,
                                 destinations);

    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        cv::imwrite(dest_paths[current_cam], destinations[current_cam]);
    }
    output_file.close();
}