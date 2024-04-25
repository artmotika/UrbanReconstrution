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

void ColorTransferMeanSamePolygons::setInputCams(pcl::texture_mapping::CameraVector &input_cams) {
    cams = input_cams;
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

void ColorTransferMeanSamePolygons::transferMeanColorBetweenPolygons(int cur_cam,
                                                                 vector <PolygonTextureCoords> &camToPolygonTextureCoords,
                                                                 vector <vector <int>> &meshFaceIndexMapInputMeshesFullToPart,
                                                                 vector <vector <int>> &meshFaceIndexMapInputMeshesPartToFull,
                                                                 vector <cv::Mat> &textures,
                                                                 cv::Mat &dest_target, std::ofstream & output_file,
                                                                 vector <cv::Mat> &destinations) {
    double lower_bound_area = 0.0;
    vector <int> area_cams_idx;

    PolygonTextureCoords polygonTextureCoordsTarget = camToPolygonTextureCoords[cur_cam];
    double area_target = Geometry_pcl::triangle_area(polygonTextureCoordsTarget.a, polygonTextureCoordsTarget.b, polygonTextureCoordsTarget.c);
    vector <double> quality_metric(number_cams);

    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        if (current_cam == cur_cam) continue;
        PolygonTextureCoords polygonTextureCoords = camToPolygonTextureCoords[current_cam];
        double area_src = Geometry_pcl::triangle_area(polygonTextureCoords.a, polygonTextureCoords.b, polygonTextureCoords.c);
        if (lower_bound_area < area_src) {
            quality_metric[current_cam] = area_src / area_target;
            if (quality_metric[current_cam] < 0.333333) continue;
            area_cams_idx.push_back(current_cam);
        }
    }
    if (area_cams_idx.size() == 0) return;

    int t2_x0 = int(polygonTextureCoordsTarget.a.x * texture_width);
    int t2_y0 = int(texture_height - polygonTextureCoordsTarget.a.y * texture_height);
    int t2_x1 = int(polygonTextureCoordsTarget.b.x * texture_width);
    int t2_y1 = int(texture_height - polygonTextureCoordsTarget.b.y * texture_height);
    int t2_x2 = int(polygonTextureCoordsTarget.c.x * texture_width);
    int t2_y2 = int(texture_height - polygonTextureCoordsTarget.c.y * texture_height);
    int t2_maxX = std::max( t2_x0, std::max( t2_x1, t2_x2) );
    int t2_maxY = std::max( t2_y0, std::max( t2_y1, t2_y2) );
    int t2_minX = std::min( t2_x0, std::min( t2_x1, t2_x2) );
    int t2_minY = std::min( t2_y0, std::min( t2_y1, t2_y2) );

    // Получаем средние значения цветов для целевого изображения
    int blueSumTarget = 0;
    int greenSumTarget = 0;
    int redSumTarget = 0;
    int countTarget = 0;
    for (int x = t2_minX; x <= t2_maxX; x++) {
        for (int y = t2_minY; y <= t2_maxY; y++) {
            pcl::PointXY pt(float(x)/float(texture_width), float(texture_height - y)/float(texture_height));
            if (checkPointInsideTriangle(polygonTextureCoordsTarget.a, polygonTextureCoordsTarget.b, polygonTextureCoordsTarget.c, pt)) {
                cv::Vec3b pixel = textures[cur_cam].at<cv::Vec3b>(y, x);
                blueSumTarget += int(pixel[0]); // синий
                greenSumTarget += int(pixel[1]); // зеленый
                redSumTarget += int(pixel[2]); // красный
                countTarget++;
            }
        }
    }

    if (countTarget == 0
        || blueSumTarget + greenSumTarget + redSumTarget == 0) {
        return;
    }

    double meanBlueTarget = double(blueSumTarget) / double(countTarget);
    double meanGreenTarget = double(greenSumTarget) / double(countTarget);
    double meanRedTarget = double(redSumTarget) / double(countTarget);

    // Получаем средние значения цветов для исходного изображения
    double blueSumSrcAll = 0.0;
    double greenSumSrcAll = 0.0;
    double redSumSrcAll = 0.0;
    double countSrcAll = 0.0;
    for (int cam_idx : area_cams_idx) {
        int blueSumSrc = 0;
        int greenSumSrc = 0;
        int redSumSrc = 0;
        int countSrc = 0;
        PolygonTextureCoords polygonTextureCoordsSrc = camToPolygonTextureCoords[cam_idx];
        int t1_x0 = int(polygonTextureCoordsSrc.a.x * texture_width);
        int t1_y0 = int(texture_height - polygonTextureCoordsSrc.a.y * texture_height);
        int t1_x1 = int(polygonTextureCoordsSrc.b.x * texture_width);
        int t1_y1 = int(texture_height - polygonTextureCoordsSrc.b.y * texture_height);
        int t1_x2 = int(polygonTextureCoordsSrc.c.x * texture_width);
        int t1_y2 = int(texture_height - polygonTextureCoordsSrc.c.y * texture_height);
        int t1_maxX = std::max( t1_x0, std::max( t1_x1, t1_x2) );
        int t1_maxY = std::max( t1_y0, std::max( t1_y1, t1_y2) );
        int t1_minX = std::min( t1_x0, std::min( t1_x1, t1_x2) );
        int t1_minY = std::min( t1_y0, std::min( t1_y1, t1_y2) );
        for (int x = t1_minX; x <= t1_maxX; x++) {
            for (int y = t1_minY; y <= t1_maxY; y++) {
                // Берем MeanColor не из треугольника, а из прямоугольника, который содержит этот треугольник
                // (чтобы учесть случай неправильной позиции камеры при дальнем расстоянии)
                pcl::PointXY pt(float(x)/float(texture_width), float(texture_height - y)/float(texture_height));
                if (checkPointInsideTriangle(polygonTextureCoordsSrc.a, polygonTextureCoordsSrc.b, polygonTextureCoordsSrc.c, pt)) {
                    cv::Vec3b pixel = textures[cam_idx].at<cv::Vec3b>(y, x);
                    blueSumSrc += int(pixel[0]); // синий
                    greenSumSrc += int(pixel[1]); // зеленый
                    redSumSrc += int(pixel[2]); // красный
                    countSrc++;
//              // LOG ONLY
//                destinations[cam_idx].at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
//              // LOG ONLY
                }
            }
        }
        if (countSrc == 0) continue;
        if (meanBlueTarget == 0.0 || meanGreenTarget == 0.0 || meanRedTarget == 0.0) {
            blueSumSrcAll += quality_metric[cam_idx] * (blueSumSrc / countSrc);
            greenSumSrcAll += quality_metric[cam_idx] * (greenSumSrc / countSrc);
            redSumSrcAll += quality_metric[cam_idx] * (redSumSrc / countSrc);
            countSrcAll += quality_metric[cam_idx];
            continue;
        }
        double alphaImage = (double(blueSumSrc) / double(countSrc)) / meanBlueTarget;
        double betaImage = (double(greenSumSrc) / double(countSrc)) / meanGreenTarget;
        double gammaImage = (double(redSumSrc) / double(countSrc)) / meanRedTarget;

        // Пропускаем изображение, если оно имеет не реалистично большую разницу с исходным target изображением
        if (alphaImage <= 0.5 || alphaImage > 2.0
        || betaImage <= 0.5 || betaImage > 2.0
        || gammaImage <= 0.5 || gammaImage > 2.0) {
            continue;
        }
//        if (alphaImage <= 0.65 || alphaImage > 1.5384
//            || betaImage <= 0.65 || betaImage > 1.5384
//            || gammaImage <= 0.65 || gammaImage > 1.5384) {
//            continue;
//        }
//        if (alphaImage <= 0.7 || alphaImage > 1.4285
//            || betaImage <= 0.7 || betaImage > 1.4285
//            || gammaImage <= 0.7 || gammaImage > 1.4285) {
//            continue;
//        }
//        // LOG ONLY
//        quality_metric[cam_idx] = 1.0;
//        // LOG ONLY
        blueSumSrcAll += quality_metric[cam_idx] * (blueSumSrc / countSrc);
        greenSumSrcAll += quality_metric[cam_idx] * (greenSumSrc / countSrc);
        redSumSrcAll += quality_metric[cam_idx] * (redSumSrc / countSrc);
        countSrcAll += quality_metric[cam_idx];
    }

    if (countSrcAll == 0.0
        || blueSumSrcAll + greenSumSrcAll + redSumSrcAll < std::numeric_limits<float>::epsilon()) {
        return;
    }

    // Рассчитываем коэффициенты для смешивания цветов
    double alpha;
    double beta;
    double gamma;
    double meanBlueSrc = double(blueSumSrcAll) / countSrcAll;
    double meanGreenSrc = double(greenSumSrcAll) / countSrcAll;
    double meanRedSrc = double(redSumSrcAll) / countSrcAll;

    if (blueSumSrcAll == 0 || blueSumTarget == 0) {
        alpha = 1.0;
    } else {
        alpha = meanBlueSrc / meanBlueTarget;
    }
    if (greenSumSrcAll == 0 || greenSumTarget == 0) {
        beta = 1.0;
    } else {
        beta = meanGreenSrc / meanGreenTarget;
    }
    if (redSumSrcAll == 0 || redSumTarget == 0) {
        gamma = 1.0;
    } else {
        gamma = meanRedSrc / meanRedTarget;
    }

    // Нереалистично маленькие значения, ошибка в параметрах позы камер
//    if (alpha <= 0.65 || beta <= 0.65 || gamma <= 0.65 || alpha > 1.5384 || beta > 1.5384 || gamma > 1.5384) {
//        return;
//    }
//    if (alpha <= 0.7 || beta <= 0.7 || gamma <= 0.7 || alpha > 1.4285 || beta > 1.4285 || gamma > 1.4285) {
//        return;
//    }
    // Нереалистично маленькие значения, ошибка в параметрах позы камер
    if (alpha <= 0.5 || beta <= 0.5 || gamma <= 0.5 || alpha > 2.0 || beta > 2.0 || gamma > 2.0) {
        return;
    }

//    cout << "meanBlueSrc: " << meanBlueSrc << " meanBlueTarget: " << meanBlueTarget << " alpha: " << alpha << endl;
//    cout << "meanGreenSrc: " << meanGreenSrc << " meanGreenTarget: " << meanGreenTarget << " beta: " << beta << endl;
//    cout << "meanRedSrc: " << meanRedSrc << " meanRedTarget: " << meanRedTarget << " gamma: " << gamma << endl;

    for (int x = t2_minX; x <= t2_maxX; x++) {
        for (int y = t2_minY; y <= t2_maxY; y++) {
            pcl::PointXY pt(float(x)/float(texture_width), float(texture_height - y)/float(texture_height));
            if (checkPointInsideTriangle(polygonTextureCoordsTarget.a, polygonTextureCoordsTarget.b, polygonTextureCoordsTarget.c, pt)) {
                cv::Vec3b pixel = dest_target.at<cv::Vec3b>(y, x);
                pixel[0] = saturate_cast<uchar>(double(pixel[0]) * alpha);
                pixel[1] = saturate_cast<uchar>(double(pixel[1]) * beta);
                pixel[2] = saturate_cast<uchar>(double(pixel[2]) * gamma);
                dest_target.at<cv::Vec3b>(y, x) = pixel;
//                dest_target.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
            }
        }
    }
}

void ColorTransferMeanSamePolygons::transferColorBetweenTpBorder(vector <vector <int>> &meshFaceIndexMapMainMeshFullToPart,
                                                          vector <vector <int>> &meshFaceIndexMapMainMeshPartToFull,
                                                          vector <vector <int>> &meshFaceIndexMapInputMeshesFullToPart,
                                                          vector <vector <int>> &meshFaceIndexMapInputMeshesPartToFull,
                                                          vector <cv::Mat> &masks,
                                                          vector <cv::Mat> &textures,
                                                          vector <cv::Mat> &destinations) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2(main_mesh.cloud, *input_cloud);
    urban_rec::Texturing_mapping texturing_mapping = urban_rec::Texturing_mapping(2000, 2000);
    std::ofstream output_file("../example_mini5/log/log3.txt");
    // LOG ONLY
    vector <vector <int>> c (number_cams, vector <int> (number_cams, 0));
    // LOG ONLY
    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        cv::Mat dest_target = destinations[current_cam];
        for (int face_idx = 0; face_idx < main_mesh.tex_polygons[current_cam].size(); face_idx++) {
//            if (border_cams_to_face_idx[current_cam][face_idx]) {
                int global_target_face_idx = meshFaceIndexMapMainMeshPartToFull[current_cam][face_idx];
                int target_face_idx = meshFaceIndexMapInputMeshesFullToPart[current_cam][global_target_face_idx];
                if (target_face_idx == -1) continue;
                vector <PolygonTextureCoords> camToPolygonTextureCoords(number_cams);
//                cout << "target_face_idx: " << target_face_idx << endl;
//                cout << "target_face_idx * 3 + 2: " << target_face_idx * 3 + 2 << endl;
                // copy UV coordinates for current_cam of face with face_idx
                pcl::PointXY p0target = PointXY(input_meshes[current_cam].tex_coordinates[0][target_face_idx * 3](0),
                                          input_meshes[current_cam].tex_coordinates[0][target_face_idx * 3](1));
                pcl::PointXY p1target = PointXY(input_meshes[current_cam].tex_coordinates[0][target_face_idx * 3 + 1](0),
                                          input_meshes[current_cam].tex_coordinates[0][target_face_idx * 3 + 1](1));
                pcl::PointXY p2target = PointXY(input_meshes[current_cam].tex_coordinates[0][target_face_idx * 3 + 2](0),
                                          input_meshes[current_cam].tex_coordinates[0][target_face_idx * 3 + 2](1));
                PolygonTextureCoords polygonTextureCoordsTarget = {p0target, p1target, p2target};
                camToPolygonTextureCoords[current_cam] = polygonTextureCoordsTarget;
                for (int cam_idx = 0; cam_idx < number_cams; cam_idx++) {
                    if (cam_idx == current_cam) continue;
                    int src_face_idx = meshFaceIndexMapInputMeshesFullToPart[cam_idx][global_target_face_idx];
                    if (src_face_idx != -1) {
                        c[current_cam][cam_idx]++;
                        // copy UV coordinates for current_cam of face with j
                        pcl::PointXY p0src = PointXY(input_meshes[cam_idx].tex_coordinates[0][src_face_idx * 3](0),
                                                  input_meshes[cam_idx].tex_coordinates[0][src_face_idx * 3](1));
                        pcl::PointXY p1src = PointXY(input_meshes[cam_idx].tex_coordinates[0][src_face_idx * 3 + 1](0),
                                                  input_meshes[cam_idx].tex_coordinates[0][src_face_idx * 3 + 1](1));
                        pcl::PointXY p2src = PointXY(input_meshes[cam_idx].tex_coordinates[0][src_face_idx * 3 + 2](0),
                                                  input_meshes[cam_idx].tex_coordinates[0][src_face_idx * 3 + 2](1));
                        int mask_num = cam_idx % 6;
                        if (mask_num == 5) continue;
                        if (texturing_mapping.isFaceOnMask(p0src, p1src, p2src, masks[mask_num])) {
                            // LOG ONLY
//                            int x0 = int(p0src.x * texture_width);
//                            int y0 = int(texture_height - p0src.y * texture_height);
//                            int x1 = int(p1src.x * texture_width);
//                            int y1 = int(texture_height - p1src.y * texture_height);
//                            int x2 = int(p2src.x * texture_width);
//                            int y2 = int(texture_height - p2src.y * texture_height);
//                            int t_maxX = std::max( x0, std::max( x1, x2) );
//                            int t_maxY = std::max( y0, std::max( y1, y2) );
//                            int t_minX = std::min( x0, std::min( x1, x2) );
//                            int t_minY = std::min( y0, std::min( y1, y2) );
//                            for (int x = t_minX; x <= t_maxX; x++) {
//                                for (int y = t_minY; y <= t_maxY; y++) {
//                                    pcl::PointXY pt(float(x)/float(texture_width), float(texture_height - y)/float(texture_height));
//                                    if (checkPointInsideTriangle(p0src, p1src, p2src, pt)) {
//                                        destinations[cam_idx].at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
//                                    }
//                                }
//                            }
                            // LOG ONLY
                            continue;
                        }

                        PolygonTextureCoords polygonTextureCoordsSrc = {p0src, p1src, p2src};
                        camToPolygonTextureCoords[cam_idx] = polygonTextureCoordsSrc;
                    }
                }
                transferMeanColorBetweenPolygons(current_cam,
                                             camToPolygonTextureCoords,
                                             meshFaceIndexMapInputMeshesFullToPart,
                                             meshFaceIndexMapInputMeshesPartToFull,
                                             textures, dest_target, output_file, destinations);
//            }
        }
    }
    // LOG ONLY
    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        for (int next_cam = 0; next_cam < number_cams; next_cam++) {
            if (next_cam == current_cam) continue;
            cout << "current_cam: " << current_cam << " next_cam: " << next_cam
                 << " c: " << c[current_cam][next_cam] << endl;
        }
    }
    // LOG ONLY
    output_file.close();
}

void ColorTransferMeanSamePolygons::transfer() {
    std::ofstream output_file("../example_mini5/log/log2.txt");
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2(triangles.cloud, *input_cloud);

//    vector <vector <bool>> border_cams_to_face_idx = getBorderTp();
    vector <vector <int>> meshFaceIndexMapInputMeshesFullToPart(number_cams);
    vector <vector <int>> meshFaceIndexMapMainMeshFullToPart(number_cams);
    vector <vector <int>> meshFaceIndexMapInputMeshesPartToFull(number_cams);
    vector <vector <int>> meshFaceIndexMapMainMeshPartToFull(number_cams);

    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        FaceIndexMaps faceIndexMaps1 = FaceIndexMaps(triangles.polygons,
                                                    input_meshes[current_cam].tex_polygons[0],
                                                    input_cloud->size());
        faceIndexMaps1.getFaceIndexMaps();
        meshFaceIndexMapInputMeshesFullToPart[current_cam] = faceIndexMaps1.faceIndexMapFullToPart;
        meshFaceIndexMapInputMeshesPartToFull[current_cam] = faceIndexMaps1.faceIndexMapPartToFull;
        FaceIndexMaps faceIndexMaps2 = FaceIndexMaps(triangles.polygons,
                                                     main_mesh.tex_polygons[current_cam],
                                                     input_cloud->size());
        faceIndexMaps2.getFaceIndexMaps();
        meshFaceIndexMapMainMeshFullToPart[current_cam] = faceIndexMaps2.faceIndexMapFullToPart;
        meshFaceIndexMapMainMeshPartToFull[current_cam] = faceIndexMaps2.faceIndexMapPartToFull;
    }

//    //LOG ONLY
//    vector<set <int>> mesh_set(number_cams);
//    vector<set <int>> main_mesh_set(number_cams);
//    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
//        set<int> set1;
//        for (int global_face_idx : meshFaceIndexMapInputMeshesPartToFull[current_cam]) {
//            if (global_face_idx != -1) {
//                set1.insert(global_face_idx);
//            }
//        }
//        mesh_set[current_cam] = set1;
//        set<int> main_set;
//        for (int global_face_idx : meshFaceIndexMapMainMeshPartToFull[current_cam]) {
//            if (global_face_idx != -1) {
//                main_set.insert(global_face_idx);
//            }
//        }
//        main_mesh_set[current_cam] = main_set;
//    }
//    vector <vector <int>> c (number_cams, vector <int> (number_cams, 0));
//    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
//        for (int face_idx = 0; face_idx < main_mesh.tex_polygons[current_cam].size(); face_idx++) {
//            int global_target_face_idx = meshFaceIndexMapMainMeshPartToFull[current_cam][face_idx];
//            for (int next_cam = 0; next_cam < number_cams; next_cam++) {
//                if (next_cam == current_cam) continue;
//                int src_face_idx = meshFaceIndexMapInputMeshesFullToPart[next_cam][global_target_face_idx];
//                if (src_face_idx != -1) c[current_cam][next_cam]++;
//            }
//        }
//    }
//
//    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
//        for (int next_cam = 0; next_cam < number_cams; next_cam++) {
//            if (next_cam == current_cam) continue;
//            cout << "current_cam: " << current_cam << " next_cam: " << next_cam
//                 << " c: " << c[current_cam][next_cam] << endl;
//        }
//    }
//
//    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
//        for (int next_cam = 0; next_cam < number_cams; next_cam++) {
//            set<int> set1 = main_mesh_set[current_cam];
//            set<int> set2 = mesh_set[next_cam];
//            std::set<int> intersection;
//            // Находим пересечение между первыми двумя множествами
//            std::set_intersection(set1.begin(), set1.end(), set2.begin(), set2.end(),
//                                  std::inserter(intersection, intersection.begin()));
//            cout << "current_cam: " << current_cam << " next_cam: " << next_cam
//                 << " intersection size: " << intersection.size() << endl;
//        }
//    }
//    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
//        output_file << "current_cam: " << current_cam << endl;
//        vector <int> v1 = meshFaceIndexMapInputMeshesFullToPart[current_cam];
//        vector <int> v2 = meshFaceIndexMapInputMeshesPartToFull[current_cam];
//        vector <int> v3 = meshFaceIndexMapMainMeshFullToPart[current_cam];
//        vector <int> v4 = meshFaceIndexMapMainMeshPartToFull[current_cam];
//        for (int i = 0; i < v1.size(); i++) {
//            output_file << "meshFaceIndexMapInputMeshesFullToPart " << "full: " << i << " part: " << v1[i] << endl;
//        }
//        for (int i = 0; i < v2.size(); i++) {
//            output_file << "meshFaceIndexMapInputMeshesPartToFull " << "part: " << i << " full: " << v2[i] << endl;
//        }
//        for (int i = 0; i < v3.size(); i++) {
//            output_file << "meshFaceIndexMapMainMeshFullToPart " << "full: " << i << " part: " << v3[i] << endl;
//        }
//        for (int i = 0; i < v4.size(); i++) {
//            output_file << "meshFaceIndexMapMainMeshPartToFull " << "part: " << i << " full: " << v4[i] << endl;
//        }
//        output_file << "\n\n\n";
//    }
//    //LOG ONLY

    vector <cv::Mat> masks;
    // Сохраняем текстуры маски в матрицы из opencv
    std::ostringstream oss_masks;
    oss_masks << dir_path << "masks/mask_Xminus.jpg";
    string masks_path = oss_masks.str();
    cout << masks_path << endl;
    cv::Mat imageMask = cv::imread(masks_path, cv::IMREAD_COLOR);
    masks.push_back(imageMask);
    cv::imwrite("../example_mini5/mask_Xminus.jpg", imageMask);
    oss_masks.str("");
    oss_masks << dir_path << "masks/mask_Xplus.jpg";
    masks_path = oss_masks.str();
    cout << masks_path << endl;
    imageMask = cv::imread(masks_path, cv::IMREAD_COLOR);
    masks.push_back(imageMask);
    cv::imwrite("../example_mini5/mask_Xplus.jpg", imageMask);
    oss_masks.str("");
    oss_masks << dir_path << "masks/mask_Yminus.jpg";
    masks_path = oss_masks.str();
    cout << masks_path << endl;
    imageMask = cv::imread(masks_path, cv::IMREAD_COLOR);
    masks.push_back(imageMask);
    cv::imwrite("../example_mini5/mask_Yminus.jpg", imageMask);
    oss_masks.str("");
    oss_masks << dir_path << "masks/mask_Yplus.jpg";
    masks_path = oss_masks.str();
    cout << masks_path << endl;
    imageMask = cv::imread(masks_path, cv::IMREAD_COLOR);
    masks.push_back(imageMask);
    cv::imwrite("../example_mini5/mask_Yplus.jpg", imageMask);
    oss_masks.str("");
    oss_masks << dir_path << "masks/mask_Zminus.jpg";
    masks_path = oss_masks.str();
    cout << masks_path << endl;
    imageMask = cv::imread(masks_path, cv::IMREAD_COLOR);
    masks.push_back(imageMask);
    cv::imwrite("../example_mini5/mask_Zminus.jpg", imageMask);

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

    transferColorBetweenTpBorder(meshFaceIndexMapMainMeshFullToPart,
                                 meshFaceIndexMapMainMeshPartToFull,
                                 meshFaceIndexMapInputMeshesFullToPart,
                                 meshFaceIndexMapInputMeshesPartToFull,
                                 masks,
                                 textures,
                                 destinations);

//    // LOG ONLY
//    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
//        for (int face_idx = 0; face_idx < input_meshes[current_cam].tex_polygons[0].size(); face_idx++) {
//            pcl::PointXY a = PointXY(input_meshes[current_cam].tex_coordinates[0][face_idx * 3](0),
//                                     input_meshes[current_cam].tex_coordinates[0][face_idx * 3](1));
//            pcl::PointXY b = PointXY(input_meshes[current_cam].tex_coordinates[0][face_idx * 3 + 1](0),
//                                     input_meshes[current_cam].tex_coordinates[0][face_idx * 3 + 1](1));
//            pcl::PointXY c = PointXY(input_meshes[current_cam].tex_coordinates[0][face_idx * 3 + 2](0),
//                                     input_meshes[current_cam].tex_coordinates[0][face_idx * 3 + 2](1));
//
//            int t2_x0 = int(a.x * texture_width);
//            int t2_y0 = int(texture_height - a.y * texture_height);
//            int t2_x1 = int(b.x * texture_width);
//            int t2_y1 = int(texture_height - b.y * texture_height);
//            int t2_x2 = int(c.x * texture_width);
//            int t2_y2 = int(texture_height - c.y * texture_height);
//
//            int t2_maxX = std::max( t2_x0, std::max( t2_x1, t2_x2) );
//            int t2_maxY = std::max( t2_y0, std::max( t2_y1, t2_y2) );
//            int t2_minX = std::min( t2_x0, std::min( t2_x1, t2_x2) );
//            int t2_minY = std::min( t2_y0, std::min( t2_y1, t2_y2) );
//
//            for (int x = t2_minX; x <= t2_maxX; x++) {
//                for (int y = t2_minY; y <= t2_maxY; y++) {
//                    pcl::PointXY pt(float(x)/float(texture_width), float(texture_height - y)/float(texture_height));
//                    if (checkPointInsideTriangle(a, b, c, pt)) {
//                        destinations[current_cam].at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
//                    }
//                }
//            }
//        }
//    }
//    // LOG ONLY

    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        cv::imwrite(dest_paths[current_cam], destinations[current_cam]);
    }
    output_file.close();
}