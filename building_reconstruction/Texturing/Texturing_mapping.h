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

#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>  // defines the PCL_INSTANTIATE_PRODUCT macro
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp> // make sure to include the .hpp file

#include <opencv2/opencv.hpp>

#include "../StringUtils/PathUtils.h"

using namespace std;
using namespace pcl;

PCL_INSTANTIATE_PRODUCT(NormalEstimation, ((pcl::PointXYZRGBNormal))((pcl::Normal))
)

namespace urban_rec {
    class Texturing_mapping {
    public:
        using Camera = pcl::texture_mapping::Camera;
        using UvIndex = pcl::texture_mapping::UvIndex;

        void setInputPolygonMesh(PolygonMesh &polygon_mesh);

        PolygonMesh getInputPolygonMesh();

        tuple<pcl::TextureMesh, pcl::texture_mapping::CameraVector> textureMesh(vector <string> argv);

        vector<pcl::TextureMesh> textureMeshes(vector <string> argv);

        static bool readCamPoseFile(string filename,
                                TextureMapping<pcl::PointXYZ>::Camera &cam);

        static bool getPointUVCoords(const PointXYZ &pt, const pcl::TextureMapping<pcl::PointXYZ>::Camera &cam,
                                     PointXY &UV_coordinates);

    private:
        PolygonMesh::Ptr input_polygon_mesh{nullptr};

        void
        textureMeshwithMultipleCameras (pcl::TextureMesh &mesh,
                                        const pcl::texture_mapping::CameraVector &cameras);

        inline void
        getTriangleCircumcenterAndSize(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3,
                                       pcl::PointXY &circomcenter, double &radius);

        inline void
        getTriangleCircumcscribedCircleCentroid(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3,
                                                pcl::PointXY &circumcenter, double &radius);

        inline bool
        getPointUVCoordinates(const pcl::PointXYZ &pt, const Camera &cam, pcl::PointXY &UV_coordinates);

        inline bool
        checkPointInsideTriangle(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3,
                                 const pcl::PointXY &pt);

        inline bool
        isFaceProjected (const Camera &camera, const pcl::PointXYZ &p1, const pcl::PointXYZ &p2, const pcl::PointXYZ &p3,
                         pcl::PointXY &proj1, pcl::PointXY &proj2, pcl::PointXY &proj3);

    };
}

#endif //URBANRECONSTRUCTION_TEXTURING_MAPPING_H
