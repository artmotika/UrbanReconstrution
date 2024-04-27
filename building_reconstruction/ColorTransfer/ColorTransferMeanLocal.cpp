//
// Created by Артем Мотыка on 16.03.2024.
//

#include "ColorTransferMeanLocal.h"

using namespace cv;
using namespace std;
using namespace pcl;
using namespace Geometry_pcl;

void ColorTransferMeanLocal::setDirPath(std::string image_path) {
    dir_path = image_path;
}

string ColorTransferMeanLocal::getDirPath() {
    return dir_path;
}

void ColorTransferMeanLocal::setInputTextureMesh(pcl::TextureMesh &mesh) {
    input_mesh = mesh;
}

void ColorTransferMeanLocal::setNumberCams(int num) {
    number_cams = num;
}

int ColorTransferMeanLocal::getNumberCams() {
    return number_cams;
}

void ColorTransferMeanLocal::setNnNumber(int num) {
    nn_number = num;
}

int ColorTransferMeanLocal::getNnNumber() {
    return nn_number;
}

vector< vector <vector <vector <NeighborPolygonTextureCoordsMap>>>> ColorTransferMeanLocal::getNeighboringTp() {
    pcl::KdTreeFLANN <pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2(input_mesh.cloud, *input_cloud);
    // Для каждой камеры хранит массив, где по индексу вершины определяется индексы полигонов,
    // которые содержат данную вершину
    vector < vector <pcl::Indices>> mesh_points_idx_polygons_idx(number_cams, vector<pcl::Indices>(input_cloud->size()));
    // Заполняем индексы точек полигонов для каждой камеры
    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        for (int face_idx = 0; face_idx < input_mesh.tex_coordinates[current_cam].size() / 3; face_idx++) {
            pcl::Vertices polygon = input_mesh.tex_polygons[current_cam][face_idx];
            // Соответствие индексу вершины индексу грани в текстурированном меше
            mesh_points_idx_polygons_idx[current_cam][polygon.vertices[0]].push_back(face_idx);
            mesh_points_idx_polygons_idx[current_cam][polygon.vertices[1]].push_back(face_idx);
            mesh_points_idx_polygons_idx[current_cam][polygon.vertices[2]].push_back(face_idx);
        }
    }

    // Ищем соседние треугольники для того, чтобы перенести цвет
    kdtree.setInputCloud(input_cloud);
    pcl::Indices idxNeighbors;
    vector<float> neighborsSquaredDistance;
    int nn_faces_1 = 0;
    int nn_faces_2 = 0;
    int nn_faces_3 = 0;
    // Для кажой камеры 1 и камеры 2 храним массив, индексом которого выступают индексы граней камеры 2,
    // а элементы – массив, состоящий из граней камеры 1, смежный с гранью с индексом элемента массива
    vector <vector <vector <vector <NeighborPolygonTextureCoordsMap>>>>
        neighboring_cams_to_polygons_texture_coords(number_cams,
            vector <vector < vector <NeighborPolygonTextureCoordsMap>>>(number_cams));
    // Массивы граней камеры 1, где индекс - индекс грани камеры 2
    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        for (int cam_idx = 0; cam_idx < number_cams; cam_idx++) {
            for (int i = 0; i < number_cams; i++) {
                vector < vector <NeighborPolygonTextureCoordsMap>> neighboring_polygons_texture_coords(
                        input_mesh.tex_polygons[i].size());
                neighboring_cams_to_polygons_texture_coords[current_cam][cam_idx] = neighboring_polygons_texture_coords;
            }
        }
    }

    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        // Проходим по каждой точке в облаке для current_cam
        for (int j = 0; j < input_mesh.tex_polygons[current_cam].size(); j++) {
            int nn_faces = 0;
            // Найти центр по координатам
            pcl::Vertices polygon = input_mesh.tex_polygons[current_cam][j];
            int src_p1_idx = polygon.vertices[0];
            int src_p2_idx = polygon.vertices[1];
            int src_p3_idx = polygon.vertices[2];
            pcl::PointXYZ src_p1 = input_cloud->points[src_p1_idx];
            pcl::PointXYZ src_p2 = input_cloud->points[src_p2_idx];
            pcl::PointXYZ src_p3 = input_cloud->points[src_p3_idx];

            float epsilon = std::numeric_limits<float>::epsilon();
            pcl::PointXYZ center = getTriangleCenterOfMass(src_p1, src_p2, src_p3);
            double radius1 = euclidean_dist_between_two_points(center, src_p1);
            double radius2 = euclidean_dist_between_two_points(center, src_p2);
            double radius3 = euclidean_dist_between_two_points(center, src_p3);
            // epilon чтобы захватить в радиус все точки треугольника, иначе одна точка всегда будет не захвачена
            double radius = max(max(radius1, radius2), radius3) + epsilon;

            // Get points inside circ.circle
            if (kdtree.radiusSearch (center, radius, idxNeighbors, neighborsSquaredDistance) > 0 ) {
                // for each neighbor
                for (const auto &idxNeighbor : idxNeighbors) {
                    // Находим точку среди облаков других камер
                    for (int cam_idx = current_cam+1; cam_idx < number_cams; cam_idx++) {
                        for (int face_idx : mesh_points_idx_polygons_idx[cam_idx][idxNeighbor]) {
                            pcl::Vertices polygon = input_mesh.tex_polygons[cam_idx][face_idx];
                            // Получаем соседний треугольник из найденного индекса
                            pcl::PointXYZ neighbor_p1 = input_cloud->points[polygon.vertices[0]];
                            pcl::PointXYZ neighbor_p2 = input_cloud->points[polygon.vertices[1]];
                            pcl::PointXYZ neighbor_p3 = input_cloud->points[polygon.vertices[2]];
                            // Проверяем соседние ли это треугольники
                            pair<int, int> segmentNum = isNeighboringPolygons(src_p1, src_p2, src_p3,
                                                               neighbor_p1, neighbor_p2, neighbor_p3);
                            if (segmentNum.first != -1) {
                                nn_faces++;
                                // copy UV coordinates for current_cam of face with j
                                pcl::PointXY tc_current_cam_p0 = PointXY(input_mesh.tex_coordinates[current_cam][j * 3](0),
                                                          input_mesh.tex_coordinates[current_cam][j * 3](1));
                                pcl::PointXY tc_current_cam_p1 = PointXY(input_mesh.tex_coordinates[current_cam][j * 3 + 1](0),
                                                          input_mesh.tex_coordinates[current_cam][j * 3 + 1](1));
                                pcl::PointXY tc_current_cam_p2 = PointXY(input_mesh.tex_coordinates[current_cam][j * 3 + 2](0),
                                                          input_mesh.tex_coordinates[current_cam][j * 3 + 2](1));
                                // copy UV coordinates for cam_idx of face with face_idx
                                pcl::PointXY tc_cam_idx_p0 = PointXY(input_mesh.tex_coordinates[cam_idx][face_idx * 3](0),
                                                          input_mesh.tex_coordinates[cam_idx][face_idx * 3](1));
                                pcl::PointXY tc_cam_idx_p1 = PointXY(input_mesh.tex_coordinates[cam_idx][face_idx * 3 + 1](0),
                                                          input_mesh.tex_coordinates[cam_idx][face_idx * 3 + 1](1));
                                pcl::PointXY tc_cam_idx_p2 = PointXY(input_mesh.tex_coordinates[cam_idx][face_idx * 3 + 2](0),
                                                          input_mesh.tex_coordinates[cam_idx][face_idx * 3 + 2](1));
                                // Сохраняем камеру 1, камеру 2 и текстурные координаты 1 и 2 для переноса цвета.
                                PolygonTextureCoords textureCoords1 = {tc_current_cam_p0, tc_current_cam_p1, tc_current_cam_p2};
                                PolygonTextureCoords textureCoords2 = {tc_cam_idx_p0, tc_cam_idx_p1, tc_cam_idx_p2};
                                NeighborPolygonTextureCoordsMap neighborPolygonTextureCoordsMap = {textureCoords1, textureCoords2, segmentNum, j, face_idx};
                                neighboring_cams_to_polygons_texture_coords[current_cam][cam_idx][face_idx].push_back(neighborPolygonTextureCoordsMap);
                            }
                        }
                    }
                }
            }

            if (nn_faces == 1) nn_faces_1++;
            if (nn_faces == 2) nn_faces_2++;
            if (nn_faces == 3) nn_faces_3++;
        }
    }
    cout << "nn_faces_1: " << nn_faces_1 << " nn_faces_2: " << nn_faces_2 << " nn_faces_3: " << nn_faces_3 << endl;
    return neighboring_cams_to_polygons_texture_coords;
}

pair<int, int> ColorTransferMeanLocal::isNeighboringPolygons(pcl::PointXYZ p01, pcl::PointXYZ p02, pcl::PointXYZ p03,
                                                   pcl::PointXYZ p11, pcl::PointXYZ p12, pcl::PointXYZ p13) {
    // Проверка на то, что сторона второго треугольника лежит на стороне первого треугольника
    if (isSegmentOnSegment(p01, p02, p11, p12)) return make_pair(0, 0);
    if (isSegmentOnSegment(p01, p02, p12, p13)) return make_pair(0, 1);
    if (isSegmentOnSegment(p01, p02, p13, p11)) return make_pair(0, 2);

    if (isSegmentOnSegment(p02, p03, p11, p12)) return make_pair(1, 0);
    if (isSegmentOnSegment(p02, p03, p12, p13)) return make_pair(1, 1);
    if (isSegmentOnSegment(p02, p03, p13, p11)) return make_pair(1, 2);

    if (isSegmentOnSegment(p03, p01, p11, p12)) return make_pair(2, 0);
    if (isSegmentOnSegment(p03, p01, p12, p13)) return make_pair(2, 1);
    if (isSegmentOnSegment(p03, p01, p13, p11)) return make_pair(2, 2);

    // Проверка на то, что вершина второго треугольника равна вершине первого треугольника
    if (arePointsEqual(p01, p11)) return make_pair(3, 3);
    if (arePointsEqual(p01, p12)) return make_pair(3, 4);
    if (arePointsEqual(p01, p13)) return make_pair(3, 5);

    if (arePointsEqual(p02, p11)) return make_pair(4, 3);
    if (arePointsEqual(p02, p12)) return make_pair(4, 4);
    if (arePointsEqual(p02, p13)) return make_pair(4, 5);

    if (arePointsEqual(p03, p11)) return make_pair(5, 3);
    if (arePointsEqual(p03, p12)) return make_pair(5, 4);
    if (arePointsEqual(p03, p13)) return make_pair(5, 5);

    return make_pair(-1, -1);
}

void ColorTransferMeanLocal::transferColorBetweenPolygons(NeighborPolygonTextureCoordsMap neighborPolygonTextureCoordsMap,
                                                          cv::Mat & texture_src,
                                                          cv::Mat & texture_target,
                                                          cv::Mat & dest_target, int c) {
    if (c > 0) return;
    c++;
    PolygonTextureCoords texture_coords1 = neighborPolygonTextureCoordsMap.textureCoords1;
    PolygonTextureCoords texture_coords2 = neighborPolygonTextureCoordsMap.textureCoords2;
    pair<int, int> segmentNum = neighborPolygonTextureCoordsMap.segmentNum;
    int face_idx1 = neighborPolygonTextureCoordsMap.face_idx1;
    int face_idx2 = neighborPolygonTextureCoordsMap.face_idx2;

    int t1_x0 = int(texture_coords1.a.x * texture_width);
    int t1_y0 = int(texture_height - texture_coords1.a.y * texture_height);
    int t1_x1 = int(texture_coords1.b.x * texture_width);
    int t1_y1 = int(texture_height - texture_coords1.b.y * texture_height);
    int t1_x2 = int(texture_coords1.c.x * texture_width);
    int t1_y2 = int(texture_height - texture_coords1.c.y * texture_height);

    int t2_x0 = int(texture_coords2.a.x * texture_width);
    int t2_y0 = int(texture_height - texture_coords2.a.y * texture_height);
    int t2_x1 = int(texture_coords2.b.x * texture_width);
    int t2_y1 = int(texture_height - texture_coords2.b.y * texture_height);
    int t2_x2 = int(texture_coords2.c.x * texture_width);
    int t2_y2 = int(texture_height - texture_coords2.c.y * texture_height);

    int t1_maxX = std::max( t1_x0, std::max( t1_x1, t1_x2) );
    int t1_maxY = std::max( t1_y0, std::max( t1_y1, t1_y2) );
    int t1_minX = std::min( t1_x0, std::min( t1_x1, t1_x2) );
    int t1_minY = std::min( t1_y0, std::min( t1_y1, t1_y2) );

    int t2_maxX = std::max( t2_x0, std::max( t2_x1, t2_x2) );
    int t2_maxY = std::max( t2_y0, std::max( t2_y1, t2_y2) );
    int t2_minX = std::min( t2_x0, std::min( t2_x1, t2_x2) );
    int t2_minY = std::min( t2_y0, std::min( t2_y1, t2_y2) );

    // Получаем средние значения цветов для исходного и целевого изображения
    int blueSumSrc = 0;
    int greenSumSrc = 0;
    int redSumSrc = 0;
    int countSrc = 0;

    for (int x = t1_minX; x <= t1_maxX; x++) {
        for (int y = t1_minY; y <= t1_maxY; y++) {
            // Берем MeanColor не из треугольника, а из прямоугольника, который содержит этот треугольник
            // (чтобы учесть случай неправильной позиции камеры при дальнем расстоянии)
//            pcl::PointXY pt(float(x)/float(texture_width), float(texture_height - y)/float(texture_height));
//            if (checkPointInsideTriangle(texture_coords1.a, texture_coords1.b, texture_coords1.c, pt)) {
                cv::Vec3b pixel = texture_src.at<cv::Vec3b>(y, x);
                blueSumSrc += int(pixel[0]); // синий
                greenSumSrc += int(pixel[1]); // зеленый
                redSumSrc += int(pixel[2]); // красный
                countSrc++;
//            }
        }
    }

    int blueSumTarget = 0;
    int greenSumTarget = 0;
    int redSumTarget = 0;
    int countTarget = 0;

    for (int x = t2_minX; x <= t2_maxX; x++) {
        for (int y = t2_minY; y <= t2_maxY; y++) {
//            pcl::PointXY pt(float(x)/float(texture_width), float(texture_height - y)/float(texture_height));
//            if (checkPointInsideTriangle(texture_coords2.a, texture_coords2.b, texture_coords2.c, pt)) {
                cv::Vec3b pixel = texture_target.at<cv::Vec3b>(y, x);
                blueSumTarget += int(pixel[0]); // синий
                greenSumTarget += int(pixel[1]); // зеленый
                redSumTarget += int(pixel[2]); // красный
                countTarget++;
//            }
        }
    }

    if (countTarget == 0 || countSrc == 0
        || blueSumSrc + greenSumSrc + redSumSrc == 0
        || blueSumTarget + greenSumTarget + redSumTarget == 0) {
        return;
    }
    // Рассчитываем коэффициенты для смешивания цветов
    double alpha;
    double beta;
    double gamma;
    double meanBlueSrc = double(blueSumSrc) / double(countSrc);
    double meanBlueTarget = double(blueSumTarget) / double(countTarget);
    double meanGreenSrc = double(greenSumSrc) / double(countSrc);
    double meanGreenTarget = double(greenSumTarget) / double(countTarget);
    double meanRedSrc = double(redSumSrc) / double(countSrc);
    double meanRedTarget = double(redSumTarget) / double(countTarget);
    if (blueSumSrc == 0) {
        alpha = 1.0;
    } else {
        alpha = meanBlueSrc / meanBlueTarget;
    }
    if (greenSumSrc == 0) {
        beta = 1.0;
    } else {
        beta = meanGreenSrc / meanGreenTarget;
    }
    if (redSumSrc == 0) {
        gamma = 1.0;
    } else {
        gamma = meanRedSrc / meanRedTarget;
    }

    for (int x = t2_minX; x <= t2_maxX; x++) {
        for (int y = t2_minY; y <= t2_maxY; y++) {
            pcl::PointXY pt(float(x)/float(texture_width), float(texture_height - y)/float(texture_height));
            if (checkPointInsideTriangle(texture_coords2.a, texture_coords2.b, texture_coords2.c, pt)) {
                cv::Vec3b pixel = texture_target.at<cv::Vec3b>(y, x);
                pixel[0] = saturate_cast<uchar>(double(pixel[0]) * alpha);
                pixel[1] = saturate_cast<uchar>(double(pixel[1]) * beta);
                pixel[2] = saturate_cast<uchar>(double(pixel[2]) * gamma);
                dest_target.at<cv::Vec3b>(y, x) = pixel;
            }
        }
    }
}

void ColorTransferMeanLocal::transferColorBetweenTpBorder(vector< vector <vector <vector <NeighborPolygonTextureCoordsMap>>>> &neighboring_cams_to_polygons_texture_coords,
                                                    vector <cv::Mat> &textures, vector <cv::Mat> &destinations,
                                                    vector <string> &dest_paths) {
    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        for (int cam_idx = current_cam + 1; cam_idx < number_cams; cam_idx++) {
            cv::Mat texture_src = textures[current_cam];
            cv::Mat texture_target = textures[cam_idx];
            cv::Mat dest_target = destinations[cam_idx];
            for (int face_idx = 0; face_idx < input_mesh.tex_polygons[cam_idx].size(); face_idx++) {
                vector<NeighborPolygonTextureCoordsMap> neighboring_polygons_texture_coords =
                        neighboring_cams_to_polygons_texture_coords[current_cam][cam_idx][face_idx];
                int c = 0;
                for (NeighborPolygonTextureCoordsMap neighborPolygonTextureCoordsMap
                        : neighboring_polygons_texture_coords) {
                    transferColorBetweenPolygons(neighborPolygonTextureCoordsMap, texture_src, texture_target, dest_target, c);
                }
            }
        }
    }
}

void ColorTransferMeanLocal::transfer() {
    vector <vector <vector <vector <NeighborPolygonTextureCoordsMap>>>> neighboring_cams_to_polygons_texture_coords = getNeighboringTp();

    vector <cv::Mat> textures;
    vector <cv::Mat> destinations;
    vector <string> dest_paths;
    // Сохраняем текстуры изображения в матрицы из opencv
    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        // Создание отдельных файлов-текстур
        std::ostringstream oss_in;
        std::ostringstream oss_dest;

        oss_in << dir_path << input_mesh.tex_materials[current_cam].tex_file;
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

    transferColorBetweenTpBorder(neighboring_cams_to_polygons_texture_coords, textures, destinations, dest_paths);

    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        cv::imwrite(dest_paths[current_cam], destinations[current_cam]);
    }
}

void colorTransfer(Mat& src, Mat& target) {
    // Получаем средние значения цветов для исходного и целевого изображения
    Scalar srcMean = mean(src);
    Scalar targetMean = mean(target);

    // Рассчитываем коэффициенты для смешивания цветов
    double alpha = targetMean[0] / srcMean[0];
    double beta = targetMean[1] / srcMean[1];
    double gamma = targetMean[2] / srcMean[2];

    // Применяем коэффициенты к каждому пикселю в исходном изображении
    for (int i = 0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            Vec3b pixel = src.at<Vec3b>(i, j);
            pixel[0] = saturate_cast<uchar>(pixel[0] * alpha);
            pixel[1] = saturate_cast<uchar>(pixel[1] * beta);
            pixel[2] = saturate_cast<uchar>(pixel[2] * gamma);
            src.at<Vec3b>(i, j) = pixel;
        }
    }
}
