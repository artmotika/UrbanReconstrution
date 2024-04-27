#ifndef URBAN_RECONSTRUCTION_FACEINDEXMAPS_H
#define URBAN_RECONSTRUCTION_FACEINDEXMAPS_H

#include <iostream>
#include <vector>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/texture_mapping.h>

using namespace std;
using namespace pcl;

class FaceIndexMaps {
public:
    FaceIndexMaps(vector <pcl::Vertices> &input_polygons,
                  vector <pcl::Vertices> &input_tex_polygons,
                  int input_num_points)
                  : polygons(input_polygons), tex_polygons(input_tex_polygons) {
        faceIndexMapFullToPart.assign(polygons.size(), -1);
        faceIndexMapPartToFull.assign(tex_polygons.size(), -1);
        setInputNumPoints(input_num_points);
    }

    void setInputNumPoints(int input_num_points);

    void setInputPolygons(vector <pcl::Vertices> &input_polygons);

    void setInputTexturePolygons(vector <pcl::Vertices> &input_tex_polygons);

    void getFaceIndexMaps();

    vector <int> faceIndexMapFullToPart;
    vector <int> faceIndexMapPartToFull;

private:
    vector <pcl::Vertices> &polygons;
    vector <pcl::Vertices> &tex_polygons;
    int num_points;
};

#endif //URBAN_RECONSTRUCTION_FACEINDEXMAPS_H
