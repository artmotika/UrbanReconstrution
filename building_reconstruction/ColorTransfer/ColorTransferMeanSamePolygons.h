#ifndef URBAN_RECONSTRUCTION_COLORTRANSFERMEANSAMEPOLYGONS_H
#define URBAN_RECONSTRUCTION_COLORTRANSFERMEANSAMEPOLYGONS_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
#include <opencv2/opencv.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/texture_mapping.h>
#include <string>
#include <chrono>

#include "../StringUtils/PathUtils.h"
#include "../Geometry/Geometry_pcl.h"
#include "ColorTransferMeanLocal.h"
#include "../Texturing/TexturingMapping.h"
#include "../Texturing/FaceIndexMaps.h"

class ColorTransferMeanSamePolygons {
public:
    using Camera = pcl::texture_mapping::Camera;

    ColorTransferMeanSamePolygons(pcl::PolygonMesh &in_triangles, pcl::TextureMesh &mesh,
                                  pcl::texture_mapping::CameraVector &cams,
                                  std::vector <pcl::TextureMesh> &meshes,
                                  std::string dir_image_path)
            : triangles(in_triangles), main_mesh(mesh), input_meshes(meshes) {
        setDirPath(dir_image_path);
        setInputCams(cams);
        setNumberCams(mesh.tex_materials.size() - 1);
    }

    void setDirPath(std::string image_path);

    std::string getDirPath();

    void setNumberCams(int num);

    int getNumberCams();

    void setInputPolygonMesh(pcl::PolygonMesh &mesh);

    void setInputTextureMesh(pcl::TextureMesh &mesh);

    void setInputCams(pcl::texture_mapping::CameraVector &input_cams);

    void setInputTextureMeshes(std::vector <pcl::TextureMesh> &meshes);

    void setLowerBoundArea(double lower_bound);

    void setMinQualityMetric(double min_quality_metric);

    void setAlphaOneSourceUpperBound(double upper_bound);

    void setAlphaOneSourceLowerBound(double lower_bound);

    void setBetaOneSourceUpperBound(double upper_bound);

    void setBetaOneSourceLowerBound(double lower_bound);

    void setGammaOneSourceUpperBound(double upper_bound);

    void setGammaOneSourceLowerBound(double lower_bound);

    void setAlphaAllSourceUpperBound(double upper_bound);

    void setAlphaAllSourceLowerBound(double lower_bound);

    void setBetaAllSourceUpperBound(double upper_bound);

    void setBetaAllSourceLowerBound(double lower_bound);

    void setGammaAllSourceUpperBound(double upper_bound);

    void setGammaAllSourceLowerBound(double lower_bound);

    std::vector <std::vector<bool>> getBorderTp();

    void transfer();

private:
    pcl::PolygonMesh &triangles;
    pcl::TextureMesh &main_mesh;
    pcl::texture_mapping::CameraVector cams;
    std::vector <pcl::TextureMesh> &input_meshes;
    std::string dir_path;
    int number_cams;
    int texture_height;
    int texture_width;
    double lower_bound_area = 0.0;
    double min_quality_metric = 0.333333;
    double alpha_one_source_upper_bound = 2.0;
    double alpha_one_source_lower_bound = 0.5;
    double beta_one_source_upper_bound = 2.0;
    double beta_one_source_lower_bound = 0.5;
    double gamma_one_source_upper_bound = 2.0;
    double gamma_one_source_lower_bound = 0.5;
    double alpha_all_source_upper_bound = 2.0;
    double alpha_all_source_lower_bound = 0.5;
    double beta_all_source_upper_bound = 2.0;
    double beta_all_source_lower_bound = 0.5;
    double gamma_all_source_upper_bound = 2.0;
    double gamma_all_source_lower_bound = 0.5;

    void transferMeanColorBetweenPolygons(int cur_cam,
                                          std::vector <PolygonTextureCoords> &camToPolygonTextureCoords,
                                          std::vector <std::vector<int>> &meshFaceIndexMapInputMeshesFullToPart,
                                          std::vector <std::vector<int>> &meshFaceIndexMapInputMeshesPartToFull,
                                          std::vector <cv::Mat> &textures,
                                          cv::Mat &dest_target,
                                          std::vector <cv::Mat> &destinations);

    void transferColorBetweenTp(std::vector <std::vector<int>> &meshFaceIndexMapMainMeshFullToPart,
                                std::vector <std::vector<int>> &meshFaceIndexMapMainMeshPartToFull,
                                std::vector <std::vector<int>> &meshFaceIndexMapInputMeshesFullToPart,
                                std::vector <std::vector<int>> &meshFaceIndexMapInputMeshesPartToFull,
                                std::vector <cv::Mat> &masks,
                                std::vector <cv::Mat> &textures,
                                std::vector <cv::Mat> &destinations);
};


#endif //URBAN_RECONSTRUCTION_COLORTRANSFERMEANSAMEPOLYGONS_H
