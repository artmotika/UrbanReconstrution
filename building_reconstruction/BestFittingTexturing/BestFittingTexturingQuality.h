#ifndef URBAN_RECONSTRUCTION_BESTFITTINGTEXTURINGQUALITY_H
#define URBAN_RECONSTRUCTION_BESTFITTINGTEXTURINGQUALITY_H

#include "../Texturing/FaceIndexMaps.h"
#include "../Geometry/Geometry_pcl.h"
#include "../Texturing/Texturing_mapping.h"

class BestFittingTexturingQuality {
public:
    BestFittingTexturingQuality(pcl::PolygonMesh &in_triangles,
                                vector<pcl::TextureMesh> &meshes)
                                : triangles(in_triangles), input_meshes(meshes) {
        setNumberCams(input_meshes.size());
    }

    void setNumberCams(int num);

    int getNumberCams();

    void setInputPolygonMesh(pcl::PolygonMesh &mesh);

    void setInputTextureMeshes(vector<pcl::TextureMesh> &meshes);

    pcl::TextureMesh fit();

private:
    pcl::PolygonMesh &triangles;
    vector<pcl::TextureMesh> &input_meshes;
    int number_cams;
};

#endif //URBAN_RECONSTRUCTION_BESTFITTINGTEXTURINGQUALITY_H
