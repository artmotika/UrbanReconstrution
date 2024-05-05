#ifndef URBAN_RECONSTRUCTION_FACEINDEXMAPS_H
#define URBAN_RECONSTRUCTION_FACEINDEXMAPS_H

#include <iostream>
#include <vector>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/texture_mapping.h>

class FaceIndexMaps {
public:
    FaceIndexMaps(std::vector <pcl::Vertices> &input_polygons,
                  std::vector <pcl::Vertices> &input_tex_polygons,
                  int input_num_points)
                  : polygons(input_polygons), tex_polygons(input_tex_polygons) {
        faceIndexMapFullToPart.assign(polygons.size(), -1);
        faceIndexMapPartToFull.assign(tex_polygons.size(), -1);
        setInputNumPoints(input_num_points);
    }

    void setInputNumPoints(int input_num_points);

    void setInputPolygons(std::vector <pcl::Vertices> &input_polygons);

    void setInputTexturePolygons(std::vector <pcl::Vertices> &input_tex_polygons);

    void getFaceIndexMaps();

    std::vector <int> faceIndexMapFullToPart;
    std::vector <int> faceIndexMapPartToFull;

private:
    std::vector <pcl::Vertices> &polygons;
    std::vector <pcl::Vertices> &tex_polygons;
    int num_points;
};

#endif //URBAN_RECONSTRUCTION_FACEINDEXMAPS_H
