#ifndef URBANRECONSTRUCTION_TEXTURING_MAPPING_H
#define URBANRECONSTRUCTION_TEXTURING_MAPPING_H

#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <cmath>
#include <math.h> /* modf */

#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>  // defines the PCL_INSTANTIATE_PRODUCT macro
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp> // make sure to include the .hpp file

#include <opencv2/opencv.hpp>

#include "../StringUtils/PathUtils.h"
#include "../Geometry/Geometry_pcl.h"

using namespace std;
using namespace pcl;

PCL_INSTANTIATE_PRODUCT(NormalEstimation, ((pcl::PointXYZRGBNormal))((pcl::Normal))
)

namespace urban_rec {
    class Texturing_mapping {
    public:
        using Camera = pcl::texture_mapping::Camera;
        using UvIndex = pcl::texture_mapping::UvIndex;

        Texturing_mapping(int width, int height) {
            setTextureWidthAndHeight(width, height);
        }

        void setTextureWidthAndHeight(int width, int height);

        void setInputPolygonMesh(PolygonMesh &polygon_mesh);

        PolygonMesh getInputPolygonMesh();

        tuple<pcl::TextureMesh, pcl::texture_mapping::CameraVector> textureMesh(vector <string> argv);

        vector<pcl::TextureMesh> textureMeshes(vector <string> argv);

        static bool readCamPoseFile(string filename,
                                TextureMapping<pcl::PointXYZ>::Camera &cam);

        static bool getPointUVCoords(const PointXYZ &pt, const pcl::TextureMapping<pcl::PointXYZ>::Camera &cam,
                                     PointXY &UV_coordinates);

        bool isFaceOnMask (pcl::PointXY &proj1, pcl::PointXY &proj2, pcl::PointXY &proj3, const cv::Mat &mask);

        static bool isFaceProjectedAngleMore (const pcl::PointXYZ &p1,
                                           const pcl::PointXYZ &p2,
                                           const pcl::PointXYZ &p3,
                                           const pcl::PointXYZ &camPose,
                                           double angle,
                                           double max_dist);

        static int saveOBJFile(const std::string &file_name,
                              const pcl::TextureMesh &tex_mesh, unsigned precision);

    private:
        PolygonMesh::Ptr input_polygon_mesh{nullptr};
        int texture_width = 0;
        int texture_height = 0;
    };
}

#endif //URBANRECONSTRUCTION_TEXTURING_MAPPING_H
