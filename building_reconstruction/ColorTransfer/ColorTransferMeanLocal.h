#ifndef URBAN_RECONSTRUCTION_COLORTRANSFERMEANLOCAL_H
#define URBAN_RECONSTRUCTION_COLORTRANSFERMEANLOCAL_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
#include <opencv2/opencv.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/texture_mapping.h>
#include <string>

#include "../StringUtils/PathUtils.h"
#include "../Geometry/Geometry_pcl.h"

struct PolygonTextureCoords {
    pcl::PointXY a, b, c;
};

struct NeighborPolygonTextureCoordsMap {
    PolygonTextureCoords textureCoords1;
    PolygonTextureCoords textureCoords2;
    std::pair<int, int> segmentNum;
    int face_idx1;
    int face_idx2;
};

class ColorTransferMeanLocal {
public:
    using Camera = pcl::texture_mapping::Camera;
    ColorTransferMeanLocal(pcl::TextureMesh &mesh, std::string dir_image_path) {
        setDirPath(dir_image_path);
        setInputTextureMesh(mesh);
        setNumberCams(mesh.tex_materials.size());
    }

    void setDirPath(std::string image_path);

    std::string getDirPath();

    void setNumberCams(int num);

    int getNumberCams();

    void setNnNumber(int num);

    int getNnNumber();

    void setInputTextureMesh(pcl::TextureMesh &mesh);

    std::vector< std::vector <std::vector <std::vector <NeighborPolygonTextureCoordsMap>>>> getNeighboringTp();

    void transfer();

private:
    pcl::TextureMesh input_mesh;
    std::string dir_path;
    int nn_number = 3;
    int number_cams;
    int texture_height;
    int texture_width;
    void transferColorBetweenPolygons(NeighborPolygonTextureCoordsMap neighborPolygonTextureCoordsMap,
                                                              cv::Mat & texture_src,
                                                              cv::Mat & texture_target,
                                                              cv::Mat & dest_target, int c);
    void transferColorBetweenTpBorder(std::vector< std::vector <std::vector <std::vector <NeighborPolygonTextureCoordsMap>>>> &neighboring_cams_to_polygons_texture_coords,
    std::vector <cv::Mat> &textures, std::vector <cv::Mat> &destinations,
    std::vector <std::string> &dest_paths);
    std::pair<int, int> isNeighboringPolygons(pcl::PointXYZ p01, pcl::PointXYZ p02, pcl::PointXYZ p03,
                               pcl::PointXYZ p11, pcl::PointXYZ p12, pcl::PointXYZ p13);
};

#endif //URBAN_RECONSTRUCTION_COLORTRANSFERMEANLOCAL_H
