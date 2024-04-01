#include "Texturing/Texturing_mapping.h"
#include "IO/Io_pcl.h"
#include "Geometry/Geometry_pcl.h"
#include "Algo_reconstruction.h"
#include "Texturing/Image/Panorama2cubemap.h"
#include "Texturing/Image/Lidar2depth.h"
#include "PointClouds/Shift.h"
#include "Building_reconstruction.h"
#include "ColorTransfer/ColorTransferMeanLocal.h"
#include "ColorTransfer/ColorTransferMeanSamePolygons.h"

#include <opencv2/core/types.hpp>
#include <tuple>
#include <limits.h>

using namespace std;
using namespace pcl;
using Camera = TextureMapping<pcl::PointXYZ>::Camera;

int main() {
    urban_rec::shift_coord shift = std::make_tuple(-369780, -2681780, -240);

    //Translate 360-image to cubemap
    urban_rec::Panorama2cubemap panorama2CubemapObject = urban_rec::Panorama2cubemap(
            "../example_mini5/pano_000001_000034.jpg",
            "../example_mini5/pano_000001_000034_cubemap.jpg",
            "../example_mini5");
    panorama2CubemapObject.setShift(shift);
    panorama2CubemapObject.setFocalLength(1000.0, 1000.0);
    panorama2CubemapObject.setCsvFile("../example_mini/reference.csv", 0, 1, '\t');
    panorama2CubemapObject.transform_dir(".jpg");

//    // Shift coordinates
//    const PCLPointCloud2::Ptr input_cloud_in(new PCLPointCloud2);
//    Io_pcl::loadCloud("../example_mini/obj_shift_n_02-12.pcd", *input_cloud_in); // 02-12 -> obj_shift_n_subsample ; obj_shift_n_part_building_o.pcd
//    PCLPointCloud2::ConstPtr input_cloud = input_cloud_in;
//
//    PointCloud<PointNormal>::Ptr points_shifted = urban_rec::shiftCoord(input_cloud, shift);
//    Io_pcl::saveCloudPCD("../example_mini/obj_shift_n_02-12.pcd", *points_shifted); // 02-12 -> obj_shift_n_part_building_shifted
//
//    const PCLPointCloud2::Ptr input_cloud_shifted(new PCLPointCloud2);
//    Io_pcl::loadCloud("../example_mini/obj_shift_n_02-12_invertednormals.pcd", *input_cloud_shifted); // 02-12 -> obj_shift_n_part_building_shifted.pcd
//
//    // Apply the Poisson surface reconstruction algorithm
//    PolygonMesh poisson_mesh;
//    int poisson_depth = 10; // 10
//    int solver_divide = 8; // 8
//    int iso_divide = 10; // 8
//    float poisson_point_weight = 10.0f; // 4.0f
//    algo_rec::computePoisson(input_cloud_shifted, poisson_mesh, poisson_depth, solver_divide, iso_divide,
//                              poisson_point_weight);
//    urban_rec::Building_reconstruction building_rec;
//    poisson_mesh = building_rec.filterMeshPoissonByPoints(poisson_mesh,
//                                                              input_cloud_shifted,
//                                                              0.5);
//    Io_pcl::saveCloud("../example_mini/obj2_mesh_invertednormals.ply", poisson_mesh); // check file obj2_mesh.ply

    // Texturing Mesh
    urban_rec::Texturing_mapping texturing_mapping = urban_rec::Texturing_mapping();
//    texturing_mapping.setInputPolygonMesh(poisson_mesh);
    std::vector <std::string> argv;
    std::string input_ply_file_path = "../example_mini/obj2_mesh_invertednormals.ply";//obj2_mesh.ply
    std::string output_ply_file_path = "../example_mini5/textured.obj";
    std::string config_path = "../example_mini5/";
    argv.push_back(input_ply_file_path);
    argv.push_back(output_ply_file_path);
    argv.push_back(config_path);

    pcl::PolygonMesh triangles;
    pcl::io::loadPolygonFilePLY(argv[0], triangles);
    texturing_mapping.setInputPolygonMesh(triangles);
    tuple<pcl::TextureMesh, pcl::texture_mapping::CameraVector> res = texturing_mapping.textureMesh(argv);
    pcl::TextureMesh tm = get<0>(res);
    pcl::texture_mapping::CameraVector cams = get<1>(res);

    vector<pcl::TextureMesh> tms = texturing_mapping.textureMeshes(argv);
    ColorTransferMeanSamePolygons color_transfer_same_polygons
        = ColorTransferMeanSamePolygons(triangles, tm, tms, "../example_mini5/");
    color_transfer_same_polygons.transfer();

////    urban_rec::Lidar2depth l2dimg = urban_rec::Lidar2depth(config_path, input_ply_file_path);
////    l2dimg.createDepthImages();
    return 0;
}
