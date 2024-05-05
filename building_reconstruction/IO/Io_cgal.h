#ifndef URBANRECONSTRUCTION_IO_CGAL_H
#define URBANRECONSTRUCTION_IO_CGAL_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Timer.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <fstream>
#include <vector>
#include <string>

namespace Io_cgal {
    void readFileToPointSet(std::string filename, CGAL::Point_set_3<Kernel::Point_3, Kernel::Vector_3> *points_start);

    void saveFileFromPointSet(std::string filename, CGAL::Point_set_3<Kernel::Point_3, Kernel::Vector_3> points_start);

    void readFileToPointVector(std::string filename, std::vector<boost::tuple<Kernel::Point_3, Kernel::Vector_3, int>> *points);

    void saveFileFromPointVector(std::string filename, CGAL::Surface_mesh<Kernel::Point_3> model);
}

#endif //URBANRECONSTRUCTION_IO_CGAL_H
