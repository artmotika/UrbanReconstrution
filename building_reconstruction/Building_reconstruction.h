#ifndef URBANRECONSTRUCTION_BUILDING_RECONSTRUCTION_H
#define URBANRECONSTRUCTION_BUILDING_RECONSTRUCTION_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <fstream>
#include <utility>
#include <limits>
#include <set>
#include <iomanip>
#include <typeinfo>
#include <stack>
#include <deque>
#include <vector>
#include <string>

#include "IO/Io_pcl.h"
#include "Geometry/Geometry_pcl.h"
#include "Algo_reconstruction.h"

struct polygon_struct {
    pcl::PointXYZ p1;
    pcl::PointXYZ p2;
    pcl::PointXYZ p3;
};

namespace urban_rec {
    class Building_reconstruction {
    public:
        void setInputFile(std::string file_name);

        std::string getInputFile();

        void setInputFileSurfaces(std::string file_name);

        std::string getInputFileSurfaces();

        void setOutputFile(std::string file_name);

        std::string getOutputFile();

        void setPoissonDepth(int depth);

        int getPoissonDepth();

        void setSolverDivide(int divide);

        int getSolverDivide();

        void setIsoDivide(int divide);

        int getIsoDivide();

        void setPoissonPointWeight(float point_weight);

        float getPoissonPointWeight();

        void setConcaveHullAlpha(float alpha);

        float getConcaveHullAlpha();

        void setConcaveHullAlphaUpsample(float alpha);

        float getConcaveHullAlphaUpsample();

        void setConvexConcaveHull(bool hull_type);

        bool getConvexConcaveHull();

        void setFilterRadius(double radius);

        double getFilterRadius();

        pcl::PolygonMesh filterMeshByMesh(pcl::PolygonMesh mesh_input, pcl::PolygonMesh mesh_filter);

        pcl::PolygonMesh
        filterMeshByPoints(pcl::PolygonMesh mesh_input, const pcl::PCLPointCloud2::ConstPtr &points_filter,
                           double filter_radius);

        pcl::PolygonMesh
        filterMeshPoissonByPoints(pcl::PolygonMesh mesh_input, const pcl::PCLPointCloud2::ConstPtr &points_filter,
                                  double filter_radius);

        pcl::PolygonMesh
        filterMeshPoissonByPointsNoExtra(pcl::PolygonMesh mesh_input,
                                         const pcl::PCLPointCloud2::ConstPtr &points_filter,
                                         double filter_radius);

        pcl::PointCloud<pcl::PointXYZ>::Ptr
        filterPointsByPoints(pcl::PolygonMesh mesh_input, const pcl::PCLPointCloud2::ConstPtr &points_filter,
                             double filter_radius);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointsByMesh(pcl::PolygonMesh mesh_input);

        void upsampleMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PolygonMesh mesh, double max_polygon_size,
                          double max_polygon_side);

        std::pair<unsigned long, double>
        calculateRepeatabilityMetric(pcl::PCLPointCloud2::Ptr cloud_ideal, pcl::PCLPointCloud2::Ptr cloud_repeat,
                                     double max_mistake, std::string cloud_not_repeat_path);

        std::pair<unsigned long, double>
        calculateHoleMetric(pcl::PCLPointCloud2::Ptr cloud_ideal, pcl::PCLPointCloud2::Ptr cloud_repeat,
                            double max_mistake,
                            std::string cloud_hole_path);

        pcl::PolygonMesh reconstruct();

        pcl::PolygonMesh reconstruct2();

    private:
        std::string input_file;
        std::string input_file_surfaces;
        std::string output_file;
        int poisson_depth = 10;
        int solver_divide = 8;
        int iso_divide = 8;
        float poisson_point_weight = 4.0f;
        float coefficient_distance_filtering = 1.8;
        float concave_hull_alpha = 8.0f;
        float concave_hull_alpha_upsample = 3.0f;
        bool convex_concave_hull = true;
        double filter_radius = 0.5;

        void upsampleByMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PolygonMesh mesh);
    };
}


#endif //URBANRECONSTRUCTION_BUILDING_RECONSTRUCTION_H
