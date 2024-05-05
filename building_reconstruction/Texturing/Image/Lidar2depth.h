#ifndef URBANRECONSTRUCTION_LIDAR2DEPTH_H
#define URBANRECONSTRUCTION_LIDAR2DEPTH_H

#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/core/types.hpp>
#include <tuple>
#include <limits.h>
#include <vector>
#include <opencv2/opencv.hpp>

#include "../TexturingMapping.h"
#include "../../StringUtils/PathUtils.h"
#include "../../IO/Io_pcl.h"
#include "../../Geometry/Geometry_pcl.h"

namespace urban_rec {
    using Camera = pcl::TextureMapping<pcl::PointXYZ>::Camera;
    using PointCoordsSet = std::vector<std::tuple<
            std::tuple<pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ>,
    std::tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector2i>,
    std::tuple<bool, bool, bool>>>;
    class Lidar2depth {
    public:
        Lidar2depth(std::string base_dir_path, std::string input_mesh_file_path = "") {
            setBaseDirPath(base_dir_path);
            if (input_mesh_file_path != "") {
                setInputMeshFilePath(input_mesh_file_path);
            }
        }

        void setInputPolygonMesh(pcl::PolygonMesh polygon_mesh);

        pcl::PolygonMesh getInputPolygonMesh();

        void setInputMeshFilePath(std::string input_mesh_file_path);

        std::string getInputMeshFilePath();

        void setBaseDirPath(std::string base_dir_path);

        std::string getBaseDirPath();

        void createDepthImages();

    private:
        std::string input_mesh_file_path;
        std::string base_dir_path;
        pcl::PolygonMesh::Ptr input_polygon_mesh{nullptr};

        void readCamPoses(std::vector<Camera> *cams, std::vector<boost::filesystem::path> *filenames);

        void fillPointCoordsSet(Camera cam, PointCoordsSet *polygons);
    };
}

#endif //URBANRECONSTRUCTION_LIDAR2DEPTH_H
