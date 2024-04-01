#ifndef URBAN_RECONSTRUCTION_COLORTRANSFERMEANSAMEPOLYGONS_H
#define URBAN_RECONSTRUCTION_COLORTRANSFERMEANSAMEPOLYGONS_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
#include <opencv2/opencv.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/texture_mapping.h>
#include <string>

#include "../StringUtils/PathUtils.h"
#include "../Geometry/Geometry_pcl.h"
#include "ColorTransferMeanLocal.h"

using namespace std;
using namespace pcl;

class ColorTransferMeanSamePolygons {
public:
    using Camera = pcl::texture_mapping::Camera;
    ColorTransferMeanSamePolygons(pcl::PolygonMesh &triangles, pcl::TextureMesh &mesh, vector<pcl::TextureMesh> &meshes,
                                  std::string dir_image_path) {
        setDirPath(dir_image_path);
        setInputPolygonMesh(triangles);
        setInputTextureMesh(mesh);
        setInputTextureMeshes(meshes);
        setNumberCams(mesh.tex_materials.size()-1);
    }

    void setDirPath(std::string image_path);

    std::string getDirPath();

    void setNumberCams(int num);

    int getNumberCams();

    void setInputPolygonMesh(pcl::PolygonMesh &mesh);

    void setInputTextureMesh(pcl::TextureMesh &mesh);

    void setInputTextureMeshes(vector<pcl::TextureMesh> &meshes);

    tuple< vector <int>, vector <int>> getFaceIndexMaps(vector <pcl::Vertices> &polygons, vector <pcl::Vertices> &tex_polygons,
                                 int num_points, std::ofstream &output_file);

    vector <vector <bool>> getBorderTp();

    void transfer();

private:
    pcl::PolygonMesh triangles;
    pcl::TextureMesh main_mesh;
    vector<pcl::TextureMesh> input_meshes;
    std::string dir_path;
    int number_cams;
    int texture_height;
    int texture_width;

    void transferColorBetweenPolygons(int cur_cam,
                                      vector <PolygonTextureCoords> &camToPolygonTextureCoords,
                                      vector <vector <int>> &meshFaceIndexMapInputMeshesFullToPart,
                                      vector <vector <int>> &meshFaceIndexMapInputMeshesPartToFull,
                                      vector <cv::Mat> &textures,
                                      cv::Mat & dest_target, std::ofstream & output_file, vector <cv::Mat> &destinations);

    void transferColorBetweenTpBorder(vector< vector <bool>> &border_cams_to_face_idx,
                                      vector <vector <int>> &meshFaceIndexMapMainMeshPartToFull,
                                      vector <vector <int>> &meshFaceIndexMapInputMeshesFullToPart,
                                      vector <vector <int>> &meshFaceIndexMapInputMeshesPartToFull,
                                      vector <cv::Mat> &textures,
                                      vector <cv::Mat> &destinations);
};


#endif //URBAN_RECONSTRUCTION_COLORTRANSFERMEANSAMEPOLYGONS_H
