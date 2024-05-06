#ifndef URBANRECONSTRUCTION_SHIFT_H
#define URBANRECONSTRUCTION_SHIFT_H

#include <pcl/PCLPointCloud2.h>
#include <tuple>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace urban_rec {
    using shift_coord = std::tuple<double, double, double>;

    pcl::PointCloud<pcl::PointNormal>::Ptr
    shiftCoord(const pcl::PCLPointCloud2::ConstPtr input_cloud, shift_coord shift);

    pcl::PointCloud<pcl::PointNormal>::Ptr
    shiftCoord(const pcl::PointCloud<pcl::PointNormal>::ConstPtr input_cloud, shift_coord shift);

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    shiftCoord(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, shift_coord shift);
}

#endif //URBANRECONSTRUCTION_SHIFT_H
