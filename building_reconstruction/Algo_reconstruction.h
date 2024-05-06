#ifndef POLYGONAL_SURFACE_RECONSTRUCTION_EXAMPLES_ALGO_RECONSTRUCTION_H
#define POLYGONAL_SURFACE_RECONSTRUCTION_EXAMPLES_ALGO_RECONSTRUCTION_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>
#include "IO/Io_pcl.h"

namespace algo_rec {
    void computePoisson(const pcl::PCLPointCloud2::ConstPtr &input, pcl::PolygonMesh &output,
                        int depth, int solver_divide, int iso_divide, float point_weight);

    void computePoisson(pcl::PointCloud<pcl::PointNormal>::Ptr &input, pcl::PolygonMesh &output,
                        int depth, int solver_divide, int iso_divide, float point_weight);

    void computeGreedyTriangulation(const pcl::PCLPointCloud2::ConstPtr &input, pcl::PolygonMesh &output,
                                    double mu, double radius);

    void computeHull(const pcl::PCLPointCloud2::ConstPtr &cloud_in,
                     bool convex_concave_hull,
                     float alpha,
                     pcl::PolygonMesh &mesh_out);

    void computeHull(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_in,
                     bool convex_concave_hull,
                     float alpha,
                     pcl::PolygonMesh &mesh_out);
}

#endif //POLYGONAL_SURFACE_RECONSTRUCTION_EXAMPLES_ALGO_RECONSTRUCTION_H
