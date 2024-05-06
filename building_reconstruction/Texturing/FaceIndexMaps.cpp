#include "FaceIndexMaps.h"

using namespace std;

void FaceIndexMaps::setInputNumPoints(int input_num_points) {
    num_points = input_num_points;
}

void FaceIndexMaps::setInputPolygons(vector <pcl::Vertices> &input_polygons) {
    polygons = input_polygons;
}

void FaceIndexMaps::setInputTexturePolygons(vector <pcl::Vertices> &input_tex_polygons) {
    tex_polygons = input_tex_polygons;
}

void FaceIndexMaps::getFaceIndexMaps() {
    faceIndexMapFullToPart.assign(polygons.size(), -1);
    faceIndexMapPartToFull.assign(tex_polygons.size(), -1);
    vector < vector < tuple < int, int, int>>> pointIndexToFaceIndex(num_points);

    for (int face_idx = 0; face_idx < tex_polygons.size(); face_idx++) {
        pcl::Vertices polygon = tex_polygons[face_idx];
        pointIndexToFaceIndex[polygon.vertices[0]].push_back(
                make_tuple(face_idx, polygon.vertices[1], polygon.vertices[2]));
    }

    for (int i = 0; i < polygons.size(); i++) {
        pcl::Vertices polygon = polygons[i];
        for (tuple<int, int, int> tup: pointIndexToFaceIndex[polygon.vertices[0]]) {
            if (get<1>(tup) == polygon.vertices[1] && get<2>(tup) == polygon.vertices[2]) {
                int face_idx = get<0>(tup);
                faceIndexMapFullToPart[i] = face_idx;
                faceIndexMapPartToFull[face_idx] = i;
            }
        }
    }
}