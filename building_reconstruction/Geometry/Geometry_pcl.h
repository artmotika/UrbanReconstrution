#ifndef URBANRECONSTRUCTION_GEOMETRY_PCL_H
#define URBANRECONSTRUCTION_GEOMETRY_PCL_H

#include <cmath>
#include <typeinfo>
#include <pcl/impl/point_types.hpp>
#include <Eigen/Dense>

namespace Geometry_pcl {
    bool point_in_radius(pcl::PointXYZ p, pcl::PointXYZ center, double radius);

    bool point_in_radius(pcl::PointNormal p, pcl::PointXYZ center, double radius);

    double euclidean_dist_between_two_points(pcl::PointXYZ a, pcl::PointXYZ b);

    double euclidean_dist_between_two_points(pcl::PointXYZRGB a, pcl::PointXYZRGB b);

    double max_euclidean_dist_side_in_polygon(std::vector <pcl::PointXYZ> ps);

    double triangle_area_geron(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3);

    double min_euclidean_dist_between_point_and_polygon_points(pcl::PointXYZ p, std::vector <pcl::PointXYZ> ps);

    bool check_point_in_triangle(Eigen::Vector2i p, Eigen::Vector2i p1, Eigen::Vector2i p2, Eigen::Vector2i p3);

    bool check_point_in_triangle_vector(Eigen::Vector2i p, Eigen::Vector2i p1, Eigen::Vector2i p2, Eigen::Vector2i p3);

    double dist_between_two_points(Eigen::Vector2i p1, Eigen::Vector2i p2);

    bool arePointsCollinear(pcl::PointXYZ a, pcl::PointXYZ b, pcl::PointXYZ c);

    bool isPointInsideSegment(pcl::PointXYZ a, pcl::PointXYZ b, pcl::PointXYZ p);

    double distanceToSegment(pcl::PointXYZ a, pcl::PointXYZ b, pcl::PointXYZ p);

    bool isSegmentOnSegment(pcl::PointXYZ a, pcl::PointXYZ b, pcl::PointXYZ c, pcl::PointXYZ d);

    bool arePointsEqual(pcl::PointXYZ a, pcl::PointXYZ b);

    pcl::PointXYZ getTriangleCenterOfMass(pcl::PointXYZ a, pcl::PointXYZ b, pcl::PointXYZ c);

    bool checkPointInsideTriangle(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3,
                                  const pcl::PointXY &pt);

    double triangle_area(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3);

}


#endif //URBANRECONSTRUCTION_GEOMETRY_PCL_H
