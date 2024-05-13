#include "ColorTransferMeanSamePolygons.h"

using namespace cv;
using namespace std;
using namespace pcl;
using namespace Geometry_pcl;
using namespace boost;

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

void ColorTransferMeanSamePolygons::setInputTextureMeshes(vector <pcl::TextureMesh> &meshes) {
    input_meshes = meshes;
}

void ColorTransferMeanSamePolygons::setLowerBoundArea(double lower_bound) {
    lower_bound_area = lower_bound;
}

void ColorTransferMeanSamePolygons::setMinQualityMetric(double min_quality_metric) {
    this->min_quality_metric = min_quality_metric;
}

void ColorTransferMeanSamePolygons::setAlphaOneSourceUpperBound(double upper_bound) {
    alpha_one_source_upper_bound = upper_bound;
}

void ColorTransferMeanSamePolygons::setAlphaOneSourceLowerBound(double lower_bound) {
    alpha_one_source_lower_bound = lower_bound;
}

void ColorTransferMeanSamePolygons::setBetaOneSourceUpperBound(double upper_bound) {
    beta_one_source_upper_bound = upper_bound;
}

void ColorTransferMeanSamePolygons::setBetaOneSourceLowerBound(double lower_bound) {
    beta_one_source_lower_bound = lower_bound;
}

void ColorTransferMeanSamePolygons::setGammaOneSourceUpperBound(double upper_bound) {
    gamma_one_source_upper_bound = upper_bound;
}

void ColorTransferMeanSamePolygons::setGammaOneSourceLowerBound(double lower_bound) {
    gamma_one_source_lower_bound = lower_bound;
}

void ColorTransferMeanSamePolygons::setAlphaAllSourceUpperBound(double upper_bound) {
    alpha_all_source_upper_bound = upper_bound;
}

void ColorTransferMeanSamePolygons::setAlphaAllSourceLowerBound(double lower_bound) {
    alpha_all_source_lower_bound = lower_bound;
}

void ColorTransferMeanSamePolygons::setBetaAllSourceUpperBound(double upper_bound) {
    beta_all_source_upper_bound = upper_bound;
}

void ColorTransferMeanSamePolygons::setBetaAllSourceLowerBound(double lower_bound) {
    beta_all_source_lower_bound = lower_bound;
}

void ColorTransferMeanSamePolygons::setGammaAllSourceUpperBound(double upper_bound) {
    gamma_all_source_upper_bound = upper_bound;
}

void ColorTransferMeanSamePolygons::setGammaAllSourceLowerBound(double lower_bound) {
    gamma_all_source_lower_bound = lower_bound;
}

void ColorTransferMeanSamePolygons::setParamsFromJson(json::object json_obj) {
    if (json_obj.contains("lower_bound_area")) {
        lower_bound_area = json_obj["lower_bound_area"].as_double();
    }
    if (json_obj.contains("min_quality_metric")) {
        min_quality_metric = json_obj["min_quality_metric"].as_double();
    }
    if (json_obj.contains("alpha_one_source_upper_bound")) {
        alpha_one_source_upper_bound = json_obj["alpha_one_source_upper_bound"].as_double();
    }
    if (json_obj.contains("alpha_one_source_lower_bound")) {
        alpha_one_source_lower_bound = json_obj["alpha_one_source_lower_bound"].as_double();
    }
    if (json_obj.contains("beta_one_source_upper_bound")) {
        beta_one_source_upper_bound = json_obj["beta_one_source_upper_bound"].as_double();
    }
    if (json_obj.contains("beta_one_source_lower_bound")) {
        beta_one_source_lower_bound = json_obj["beta_one_source_lower_bound"].as_double();
    }
    if (json_obj.contains("gamma_one_source_upper_bound")) {
        gamma_one_source_upper_bound = json_obj["gamma_one_source_upper_bound"].as_double();
    }
    if (json_obj.contains("gamma_one_source_lower_bound")) {
        gamma_one_source_lower_bound = json_obj["gamma_one_source_lower_bound"].as_double();
    }
    if (json_obj.contains("alpha_all_source_upper_bound")) {
        alpha_all_source_upper_bound = json_obj["alpha_all_source_upper_bound"].as_double();
    }
    if (json_obj.contains("alpha_all_source_lower_bound")) {
        alpha_all_source_lower_bound = json_obj["alpha_all_source_lower_bound"].as_double();
    }
    if (json_obj.contains("beta_all_source_upper_bound")) {
        beta_all_source_upper_bound = json_obj["beta_all_source_upper_bound"].as_double();
    }
    if (json_obj.contains("beta_all_source_lower_bound")) {
        beta_all_source_lower_bound = json_obj["beta_all_source_lower_bound"].as_double();
    }
    if (json_obj.contains("gamma_all_source_upper_bound")) {
        gamma_all_source_upper_bound = json_obj["gamma_all_source_upper_bound"].as_double();
    }
    if (json_obj.contains("gamma_all_source_lower_bound")) {
        gamma_all_source_lower_bound = json_obj["gamma_all_source_lower_bound"].as_double();
    }
}

vector <vector<bool>> ColorTransferMeanSamePolygons::getBorderTp() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud <pcl::PointXYZ>);
    fromPCLPointCloud2(main_mesh.cloud, *input_cloud);
    // Для каждой камеры хранит массив, где по индексу вершины определяется индексы полигонов,
    // которые содержат данную вершину
    vector <vector<pcl::Indices>> mesh_points_idx_polygons_idx(number_cams, vector<pcl::Indices>(input_cloud->size()));
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

    // Для кажой камеры храним для кадого индекса грани, является ли он границей патча
    vector <vector<bool>> border_cams_to_face_idx(number_cams);

    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        vector<bool> faceToIsBorder(main_mesh.tex_polygons[current_cam].size(), false);
        border_cams_to_face_idx[current_cam] = faceToIsBorder;
    }

    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        // Проходим по каждой точке в облаке для current_cam
        for (int j = 0; j < main_mesh.tex_polygons[current_cam].size(); j++) {
            // Найти центр по координатам
            pcl::Vertices polygon = main_mesh.tex_polygons[current_cam][j];
            // Находим точку среди облаков других камер
            for (int cam_idx = current_cam + 1; cam_idx < number_cams; cam_idx++) {
                for (int vertex_idx: polygon.vertices) {
                    for (int face_idx: mesh_points_idx_polygons_idx[cam_idx][vertex_idx]) {
                        border_cams_to_face_idx[current_cam][j] = true;
                        border_cams_to_face_idx[cam_idx][face_idx] = true;
                    }
                }
            }
        }
    }

    return border_cams_to_face_idx;
}

void ColorTransferMeanSamePolygons::transferMeanColorBetweenPolygons(int cur_cam,
                                                                     vector <PolygonTextureCoords> &camToPolygonTextureCoords,
                                                                     vector <vector<int>> &meshFaceIndexMapInputMeshesFullToPart,
                                                                     vector <vector<int>> &meshFaceIndexMapInputMeshesPartToFull,
                                                                     vector <cv::Mat> &textures,
                                                                     cv::Mat &dest_target,
                                                                     vector <cv::Mat> &destinations) {
    vector<int> area_cams_idx;
    PolygonTextureCoords polygonTextureCoordsTarget = camToPolygonTextureCoords[cur_cam];
    double area_target = Geometry_pcl::triangle_area(polygonTextureCoordsTarget.a, polygonTextureCoordsTarget.b,
                                                     polygonTextureCoordsTarget.c);
    vector<double> quality_metric(number_cams);

    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        if (current_cam == cur_cam) continue;
        PolygonTextureCoords polygonTextureCoords = camToPolygonTextureCoords[current_cam];
        double area_src = Geometry_pcl::triangle_area(polygonTextureCoords.a, polygonTextureCoords.b,
                                                      polygonTextureCoords.c);
        if (lower_bound_area < area_src) {
            quality_metric[current_cam] = area_src / area_target;
            if (quality_metric[current_cam] < min_quality_metric) continue;
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
    int t2_maxX = std::max(t2_x0, std::max(t2_x1, t2_x2));
    int t2_maxY = std::max(t2_y0, std::max(t2_y1, t2_y2));
    int t2_minX = std::min(t2_x0, std::min(t2_x1, t2_x2));
    int t2_minY = std::min(t2_y0, std::min(t2_y1, t2_y2));

    // Получаем средние значения цветов для целевого изображения
    int blueSumTarget = 0;
    int greenSumTarget = 0;
    int redSumTarget = 0;
    int countTarget = 0;
    for (int x = t2_minX; x <= t2_maxX; x++) {
        for (int y = t2_minY; y <= t2_maxY; y++) {
            pcl::PointXY pt(float(x) / float(texture_width), float(texture_height - y) / float(texture_height));
            if (checkPointInsideTriangle(polygonTextureCoordsTarget.a, polygonTextureCoordsTarget.b,
                                         polygonTextureCoordsTarget.c, pt)) {
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
    for (int cam_idx: area_cams_idx) {
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
        int t1_maxX = std::max(t1_x0, std::max(t1_x1, t1_x2));
        int t1_maxY = std::max(t1_y0, std::max(t1_y1, t1_y2));
        int t1_minX = std::min(t1_x0, std::min(t1_x1, t1_x2));
        int t1_minY = std::min(t1_y0, std::min(t1_y1, t1_y2));
        for (int x = t1_minX; x <= t1_maxX; x++) {
            for (int y = t1_minY; y <= t1_maxY; y++) {
                // Берем MeanColor не из треугольника, а из прямоугольника, который содержит этот треугольник
                // (чтобы учесть случай неправильной позиции камеры при дальнем расстоянии)
                pcl::PointXY pt(float(x) / float(texture_width), float(texture_height - y) / float(texture_height));
                if (checkPointInsideTriangle(polygonTextureCoordsSrc.a, polygonTextureCoordsSrc.b,
                                             polygonTextureCoordsSrc.c, pt)) {
                    cv::Vec3b pixel = textures[cam_idx].at<cv::Vec3b>(y, x);
                    blueSumSrc += int(pixel[0]); // синий
                    greenSumSrc += int(pixel[1]); // зеленый
                    redSumSrc += int(pixel[2]); // красный
                    countSrc++;
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
        if (alphaImage <= alpha_one_source_lower_bound || alphaImage > alpha_one_source_upper_bound
            || betaImage <= beta_one_source_lower_bound || betaImage > beta_one_source_upper_bound
            || gammaImage <= gamma_one_source_lower_bound || gammaImage > gamma_one_source_upper_bound) {
            continue;
        }
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
    if (alpha <= alpha_all_source_lower_bound
        || beta <= beta_all_source_lower_bound
        || gamma <= gamma_all_source_lower_bound
        || alpha > alpha_all_source_upper_bound
        || beta > beta_all_source_upper_bound
        || gamma > gamma_all_source_upper_bound) {
        return;
    }

    for (int x = t2_minX; x <= t2_maxX; x++) {
        for (int y = t2_minY; y <= t2_maxY; y++) {
            pcl::PointXY pt(float(x) / float(texture_width), float(texture_height - y) / float(texture_height));
            if (checkPointInsideTriangle(polygonTextureCoordsTarget.a, polygonTextureCoordsTarget.b,
                                         polygonTextureCoordsTarget.c, pt)) {
                cv::Vec3b pixel = dest_target.at<cv::Vec3b>(y, x);
                pixel[0] = saturate_cast<uchar>(double(pixel[0]) * alpha);
                pixel[1] = saturate_cast<uchar>(double(pixel[1]) * beta);
                pixel[2] = saturate_cast<uchar>(double(pixel[2]) * gamma);
                dest_target.at<cv::Vec3b>(y, x) = pixel;
            }
        }
    }
}

void ColorTransferMeanSamePolygons::transferColorBetweenTp(vector <vector<int>> &meshFaceIndexMapMainMeshFullToPart,
                                                           vector <vector<int>> &meshFaceIndexMapMainMeshPartToFull,
                                                           vector <vector<int>> &meshFaceIndexMapInputMeshesFullToPart,
                                                           vector <vector<int>> &meshFaceIndexMapInputMeshesPartToFull,
                                                           vector <cv::Mat> &masks,
                                                           vector <cv::Mat> &textures,
                                                           vector <cv::Mat> &destinations) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud <pcl::PointXYZ>);
    fromPCLPointCloud2(main_mesh.cloud, *input_cloud);
    urban_rec::TexturingMapping texturing_mapping = urban_rec::TexturingMapping(texture_width, texture_height);
    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        cv::Mat dest_target = destinations[current_cam];
        for (int face_idx = 0; face_idx < main_mesh.tex_polygons[current_cam].size(); face_idx++) {
            int global_target_face_idx = meshFaceIndexMapMainMeshPartToFull[current_cam][face_idx];
            int target_face_idx = meshFaceIndexMapInputMeshesFullToPart[current_cam][global_target_face_idx];
            if (target_face_idx == -1) continue;
            vector <PolygonTextureCoords> camToPolygonTextureCoords(number_cams);
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
                                             textures, dest_target, destinations);
        }
    }
}

void ColorTransferMeanSamePolygons::transfer() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud <pcl::PointXYZ>);
    fromPCLPointCloud2(triangles.cloud, *input_cloud);

    vector <vector<int>> meshFaceIndexMapInputMeshesFullToPart(number_cams);
    vector <vector<int>> meshFaceIndexMapMainMeshFullToPart(number_cams);
    vector <vector<int>> meshFaceIndexMapInputMeshesPartToFull(number_cams);
    vector <vector<int>> meshFaceIndexMapMainMeshPartToFull(number_cams);

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

    vector <cv::Mat> masks;
    // Сохраняем текстуры маски в матрицы из opencv
    std::ostringstream oss_masks;
    oss_masks << dir_path << "masks/mask_Xminus.jpg";
    string masks_path = oss_masks.str();
    cout << masks_path << endl;
    cv::Mat imageMask = cv::imread(masks_path, cv::IMREAD_COLOR);
    masks.push_back(imageMask);
    oss_masks.str("");
    oss_masks << dir_path << "masks/mask_Xplus.jpg";
    masks_path = oss_masks.str();
    cout << masks_path << endl;
    imageMask = cv::imread(masks_path, cv::IMREAD_COLOR);
    masks.push_back(imageMask);
    oss_masks.str("");
    oss_masks << dir_path << "masks/mask_Yminus.jpg";
    masks_path = oss_masks.str();
    cout << masks_path << endl;
    imageMask = cv::imread(masks_path, cv::IMREAD_COLOR);
    masks.push_back(imageMask);
    oss_masks.str("");
    oss_masks << dir_path << "masks/mask_Yplus.jpg";
    masks_path = oss_masks.str();
    cout << masks_path << endl;
    imageMask = cv::imread(masks_path, cv::IMREAD_COLOR);
    masks.push_back(imageMask);
    oss_masks.str("");
    oss_masks << dir_path << "masks/mask_Zminus.jpg";
    masks_path = oss_masks.str();
    cout << masks_path << endl;
    imageMask = cv::imread(masks_path, cv::IMREAD_COLOR);
    masks.push_back(imageMask);

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

    transferColorBetweenTp(meshFaceIndexMapMainMeshFullToPart,
                           meshFaceIndexMapMainMeshPartToFull,
                           meshFaceIndexMapInputMeshesFullToPart,
                           meshFaceIndexMapInputMeshesPartToFull,
                           masks,
                           textures,
                           destinations);

    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        cv::imwrite(dest_paths[current_cam], destinations[current_cam]);
    }
}