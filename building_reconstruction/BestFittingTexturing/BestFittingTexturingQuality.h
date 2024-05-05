#ifndef URBAN_RECONSTRUCTION_BESTFITTINGTEXTURINGQUALITY_H
#define URBAN_RECONSTRUCTION_BESTFITTINGTEXTURINGQUALITY_H

#include <opencv2/opencv.hpp>
#include <string>
#include "../Texturing/FaceIndexMaps.h"
#include "../Geometry/Geometry_pcl.h"
#include "../Texturing/TexturingMapping.h"
#include "../consts/PCLTexMaterialConsts.h"

class BestFittingTexturingQuality {
public:
    BestFittingTexturingQuality(pcl::PolygonMesh &in_triangles,
                                std::vector<pcl::TextureMesh> &meshes)
                                : triangles(in_triangles), input_meshes(meshes) {
        setNumberCams(input_meshes.size());
    }

    void setNumberCams(int num);

    int getNumberCams();

    void setInputPolygonMesh(pcl::PolygonMesh &mesh);

    void setInputTextureMeshes(std::vector<pcl::TextureMesh> &meshes);

    pcl::TextureMesh fit(std::vector <std::string> argv);

private:
    pcl::PolygonMesh &triangles;
    std::vector<pcl::TextureMesh> &input_meshes;
    int number_cams;
};

#endif //URBAN_RECONSTRUCTION_BESTFITTINGTEXTURINGQUALITY_H
