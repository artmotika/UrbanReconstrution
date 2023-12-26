#ifndef POLYGONAL_SURFACE_RECONSTRUCTION_EXAMPLES_ALGO_RECONSTRUCTION_H
#define POLYGONAL_SURFACE_RECONSTRUCTION_EXAMPLES_ALGO_RECONSTRUCTION_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>
#include "IO/Io_pcl.h"

using namespace pcl;
using namespace pcl::console;

namespace algo_rec {
    void computePoisson(const PCLPointCloud2::ConstPtr &input, PolygonMesh &output,
                         int depth, int solver_divide, int iso_divide, float point_weight);

    void computePoisson(PointCloud<PointNormal>::Ptr &input, PolygonMesh &output,
                         int depth, int solver_divide, int iso_divide, float point_weight);

    void computeGreedyTriangulation(const PCLPointCloud2::ConstPtr &input, PolygonMesh &output,
                                      double mu, double radius);

    void computeHull(const PCLPointCloud2::ConstPtr &cloud_in,
                      bool convex_concave_hull,
                      float alpha,
                      PolygonMesh &mesh_out);

    void computeHull(const PointCloud<PointXYZ>::ConstPtr &cloud_in,
                      bool convex_concave_hull,
                      float alpha,
                      PolygonMesh &mesh_out);
}

#endif //POLYGONAL_SURFACE_RECONSTRUCTION_EXAMPLES_ALGO_RECONSTRUCTION_H
