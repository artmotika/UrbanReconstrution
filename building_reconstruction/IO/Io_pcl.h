#ifndef URBANRECONSTRUCTION_IO_PCL_H
#define URBANRECONSTRUCTION_IO_PCL_H

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <string>

namespace Io_pcl {

    bool loadCloudPLY(const std::string &filename, pcl::PCLPointCloud2 &cloud);

    bool loadCloud(const std::string &filename, pcl::PCLPointCloud2 &cloud);

    bool loadCloud(const std::string &filename, pcl::PointCloud <pcl::PointXYZ> &cloud);

    bool loadCloud(const std::string &filename, pcl::PolygonMesh &cloud);

    void saveCloud(const std::string &filename, const pcl::PolygonMesh &cloud);

    void saveCloud(std::string const &filename, pcl::PointCloud <pcl::PointXYZ> const &cloud);

    void saveCloud(std::string const &filename, pcl::PointCloud <pcl::PointNormal> const &cloud);

    void saveCloud(std::string const &filename, pcl::PCLPointCloud2 const &cloud);

    void saveCloudPCD(std::string const &filename, pcl::PointCloud <pcl::PointXYZ> const &cloud);

    void saveCloudPCD(std::string const &filename, pcl::PointCloud <pcl::PointXYZRGB> const &cloud);

    void saveCloudPCD(std::string const &filename, pcl::PointCloud <pcl::PointNormal> const &cloud);

    void saveCloudPCD(std::string const &filename, pcl::PCLPointCloud2 const &cloud);
}

#endif //URBANRECONSTRUCTION_IO_PCL_H
