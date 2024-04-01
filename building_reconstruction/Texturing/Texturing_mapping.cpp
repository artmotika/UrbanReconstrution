#include "Texturing_mapping.h"
#include "IO/Image_txt_reading.h"

using namespace urban_rec;

int saveOBJFile(const std::string &file_name,
                const pcl::TextureMesh &tex_mesh, unsigned precision) {
    if (tex_mesh.cloud.data.empty()) {
        PCL_ERROR("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
        return (-1);
    }

    // Open file
    std::ofstream fs;
    fs.precision(precision);
    fs.open(file_name.c_str());

    // Define material file
    std::string mtl_file_name = file_name.substr(0, file_name.find_last_of(".")) + ".mtl";
    // Strip path for "mtllib" command
    std::string mtl_file_name_nopath = mtl_file_name;
    mtl_file_name_nopath.erase(0, mtl_file_name.find_last_of('/') + 1);

    /* Write 3D information */
    // number of points
    int nr_points = tex_mesh.cloud.width * tex_mesh.cloud.height;
    int point_size = tex_mesh.cloud.data.size() / nr_points;

    // mesh size
    int nr_meshes = tex_mesh.tex_polygons.size();
    // number of faces for header
    int nr_faces = 0;
    for (int m = 0; m < nr_meshes; ++m)
        nr_faces += tex_mesh.tex_polygons[m].size();

    // Write the header information
    fs << "####" << std::endl;
    fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
    fs << "# Vertices: " << nr_points << std::endl;
    fs << "# Faces: " << nr_faces << std::endl;
    fs << "# Material information:" << std::endl;
    fs << "mtllib " << mtl_file_name_nopath << std::endl;
    fs << "####" << std::endl;

    // Write vertex coordinates
    fs << "# Vertices" << std::endl;
    for (int i = 0; i < nr_points; ++i) {
        int xyz = 0;
        // "v" just be written one
        bool v_written = false;
        for (size_t d = 0; d < tex_mesh.cloud.fields.size(); ++d) {
            int count = tex_mesh.cloud.fields[d].count;
            if (count == 0)
                count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
            int c = 0;
            // adding vertex
            if ((//tex_mesh.cloud.fields[d].datatype == sensor_msgs::PointField::FLOAT32) && (
                    tex_mesh.cloud.fields[d].name == "x" ||
                    tex_mesh.cloud.fields[d].name == "y" ||
                    tex_mesh.cloud.fields[d].name == "z")) {
                if (!v_written) {
                    // write vertices beginning with v
                    fs << "v ";
                    v_written = true;
                }
                float value;
                memcpy(&value,
                       &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof(float)],
                       sizeof(float));
                fs << value;
                if (++xyz == 3)
                    break;
                fs << " ";
            }
        }
        if (xyz != 3) {
            PCL_ERROR("[pcl::io::saveOBJFile] Input point cloud has no XYZ data!\n");
            return (-2);
        }
        fs << std::endl;
    }
    fs << "# " << nr_points << " vertices" << std::endl;

    // Write vertex normals
    for (int i = 0; i < nr_points; ++i) {
        int xyz = 0;
        bool v_written = false;
        for (size_t d = 0; d < tex_mesh.cloud.fields.size(); ++d) {
            int count = tex_mesh.cloud.fields[d].count;
            if (count == 0)
                count = 1; // we simply cannot tolerate 0 counts (coming from older converter code)
            int c = 0;
            // adding vertex
            if ((   tex_mesh.cloud.fields[d].name == "normal_x" ||
                    tex_mesh.cloud.fields[d].name == "normal_y" ||
                    tex_mesh.cloud.fields[d].name == "normal_z")) {
                if (!v_written) {
                    // write vertices beginning with vn
                    fs << "vn ";
                    v_written = true;
                }
                float value;
                memcpy(&value,
                       &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof(float)],
                       sizeof(float));
                fs << value;
                if (++xyz == 3)
                    break;
                fs << " ";
            }
        }
        if (xyz != 3) {
            PCL_ERROR("[pcl::io::saveOBJFile] Input point cloud has no normals!\n");
            return (-2);
        }
        fs << std::endl;
    }

    // Write vertex texture with "vt" (adding latter)
    for (int m = 0; m < nr_meshes; ++m) {
        if (tex_mesh.tex_coordinates.size() == 0)
            continue;

        PCL_INFO("%d vertex textures in submesh %d\n", tex_mesh.tex_coordinates[m].size(), m);
        fs << "# " << tex_mesh.tex_coordinates[m].size() << " vertex textures in submesh " << m << std::endl;
        for (size_t i = 0; i < tex_mesh.tex_coordinates[m].size(); ++i) {
            fs << "vt ";
            fs << tex_mesh.tex_coordinates[m][i][0] << " " << tex_mesh.tex_coordinates[m][i][1] << std::endl;
        }
    }

    int f_idx = 0;

    // int idx_vt =0;
    PCL_INFO("Writting faces...\n");
    for (int m = 0; m < nr_meshes; ++m) {
        if (m > 0)
            f_idx += tex_mesh.tex_polygons[m - 1].size();

        if (tex_mesh.tex_materials.size() != 0) {
            fs << "# The material will be used for mesh " << m << std::endl;
            fs << "usemtl " << tex_mesh.tex_materials[m].tex_name << std::endl;
            fs << "# Faces" << std::endl;
        }
        for (size_t i = 0; i < tex_mesh.tex_polygons[m].size(); ++i) {
            // Write faces with "f"
            fs << "f";
            size_t j = 0;
            // There's one UV per vertex per face, i.e., the same vertex can have
            // different UV depending on the face.
            for (j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size(); ++j) {
                unsigned int idx = tex_mesh.tex_polygons[m][i].vertices[j] + 1;
                fs << " " << idx
                   << "/" << 3 * (i + f_idx) + j + 1
                   << "/" << idx; // vertex index in obj file format starting with 1
            }
            fs << std::endl;
        }
        PCL_INFO("%d faces in mesh %d \n", tex_mesh.tex_polygons[m].size(), m);
        fs << "# " << tex_mesh.tex_polygons[m].size() << " faces in mesh " << m << std::endl;
    }
    fs << "# End of File";

    // Close obj file
    PCL_INFO("Closing obj file\n");
    fs.close();

    /* Write material defination for OBJ file*/
    // Open file
    PCL_INFO("Writing material files\n");
    // dont do it if no material to write
    if (tex_mesh.tex_materials.size() == 0)
        return (0);

    std::ofstream m_fs;
    m_fs.precision(precision);
    m_fs.open(mtl_file_name.c_str());

    // default
    m_fs << "#" << std::endl;
    m_fs << "# Wavefront material file" << std::endl;
    m_fs << "#" << std::endl;
    for (int m = 0; m < nr_meshes; ++m) {
        m_fs << "newmtl " << tex_mesh.tex_materials[m].tex_name << std::endl;
        m_fs << "Ka " << tex_mesh.tex_materials[m].tex_Ka.r << " " << tex_mesh.tex_materials[m].tex_Ka.g << " "
             << tex_mesh.tex_materials[m].tex_Ka.b
             << std::endl; // defines the ambient color of the material to be (r,g,b).
        m_fs << "Kd " << tex_mesh.tex_materials[m].tex_Kd.r << " " << tex_mesh.tex_materials[m].tex_Kd.g << " "
             << tex_mesh.tex_materials[m].tex_Kd.b
             << std::endl; // defines the diffuse color of the material to be (r,g,b).
        m_fs << "Ks " << tex_mesh.tex_materials[m].tex_Ks.r << " " << tex_mesh.tex_materials[m].tex_Ks.g << " "
             << tex_mesh.tex_materials[m].tex_Ks.b
             << std::endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
        m_fs << "d " << tex_mesh.tex_materials[m].tex_d
             << std::endl; // defines the transparency of the material to be alpha.
        m_fs << "Ns " << tex_mesh.tex_materials[m].tex_Ns
             << std::endl; // defines the shininess of the material to be s.
        m_fs << "illum " << tex_mesh.tex_materials[m].tex_illum
             << std::endl; // denotes the illumination model used by the material.
        // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
        // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
        m_fs << "map_Kd " << tex_mesh.tex_materials[m].tex_file << std::endl;
        m_fs << "###" << std::endl;
    }
    m_fs.close();
    return (0);
}

void urban_rec::Texturing_mapping::setInputPolygonMesh(pcl::PolygonMesh &polygon_mesh) {
    input_polygon_mesh = std::make_shared<pcl::PolygonMesh>(polygon_mesh);
}

pcl::PolygonMesh urban_rec::Texturing_mapping::getInputPolygonMesh() {
    return *input_polygon_mesh;
}

//bool urban_rec::Texturing_mapping::getPointUVCoords(const PointXYZ &pt, const pcl::TextureMapping<pcl::PointXYZ>::Camera &cam, PointXY &UV_coordinates) {
//    // if the point is in front of the camera
//    if (pt.z > 0)
//    {
//        // compute image center and dimension
//        double sizeX = cam.width;
//        double sizeY = cam.height;
//        double cx, cy;
//
//        if (cam.center_w > 0)
//            cx = cam.center_w;
//        else
//            cx = sizeX / 2.0;
//        if (cam.center_h > 0)
//            cy = cam.center_h;
//        else
//            cy = sizeY / 2.0;
//
//        double focal_x, focal_y;
//        if (cam.focal_length_w > 0)
//            focal_x = cam.focal_length_w;
//        else
//            focal_x = cam.focal_length;
//        if (cam.focal_length_h > 0)
//            focal_y = cam.focal_length_h;
//        else
//            focal_y = cam.focal_length;
//
//        // project point on camera's image plane
//        UV_coordinates.x = static_cast<float> ((focal_x * (pt.x / pt.z) + cx) / sizeX); //horizontal
//        UV_coordinates.y = 1.0f - static_cast<float> ((focal_y * (pt.y / pt.z) + cy) / sizeY); //vertical
//
//        // point is visible!
//        if (UV_coordinates.x >= 0.0 && UV_coordinates.x <= 1.0 && UV_coordinates.y >= 0.0 && UV_coordinates.y <= 1.0)
//            return (true); // point was visible by the camera
//    }
//
//    // point is NOT visible by the camera
//    return (false); // point was not visible by the camera
//}

bool urban_rec::Texturing_mapping::readCamPoseFile(std::string filename,
                                                      pcl::TextureMapping<pcl::PointXYZ>::Camera &cam) {
    std::ifstream myReadFile;
    myReadFile.open(filename.c_str(), ios::in);
    if (!myReadFile.is_open()) {
        PCL_ERROR("Error opening file %d\n", filename.c_str());
        return false;
    }
    myReadFile.seekg(ios::beg);

    double val;

    urban_rec::image_txt_read::go_to_line(myReadFile, 1);
    myReadFile >> val;
    cam.pose(0, 3) = val;  // TX
    myReadFile >> val;
    cam.pose(1, 3) = val;  // TY
    myReadFile >> val;
    cam.pose(2, 3) = val;  // TZ

    urban_rec::image_txt_read::go_to_line(myReadFile, 2);
    myReadFile >> val;
    cam.pose(0, 0) = val;
    myReadFile >> val;
    cam.pose(0, 1) = val;
    myReadFile >> val;
    cam.pose(0, 2) = val;

    myReadFile >> val;
    cam.pose(1, 0) = val;
    myReadFile >> val;
    cam.pose(1, 1) = val;
    myReadFile >> val;
    cam.pose(1, 2) = val;

    myReadFile >> val;
    cam.pose(2, 0) = val;
    myReadFile >> val;
    cam.pose(2, 1) = val;
    myReadFile >> val;
    cam.pose(2, 2) = val;

    // Scale
    cam.pose(3, 0) = 0.0;
    cam.pose(3, 1) = 0.0;
    cam.pose(3, 2) = 0.0;
    cam.pose(3, 3) = 1.0;

    urban_rec::image_txt_read::go_to_line(myReadFile, 5);
    myReadFile >> val;
    cam.focal_length_w = val;
    myReadFile >> val;
    cam.focal_length_h = val;
//    myReadFile >> val;
//    cam.center_w = val;
//    myReadFile >> val;
//    cam.center_h = val;
    myReadFile >> val;
    cam.height = val;
    myReadFile >> val;
    cam.width = val;

    // close file
    myReadFile.close();
    return true;
}

void
urban_rec::Texturing_mapping::textureMeshwithMultipleCameras (pcl::TextureMesh &mesh, const pcl::texture_mapping::CameraVector &cameras)
{

    if (mesh.tex_polygons.size () != 1)
        return;

    typename pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2 (mesh.cloud, *mesh_cloud);

    std::vector<pcl::Vertices> faces;

    for (int current_cam = 0; current_cam < static_cast<int> (cameras.size ()); ++current_cam)
    {
        PCL_INFO ("Processing camera %d of %d.\n", current_cam+1, cameras.size ());

        // transform mesh into camera's frame
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud (*mesh_cloud, *camera_cloud, cameras[current_cam].pose.inverse ());

        // CREATE UV MAP FOR CURRENT FACES
        pcl::PointCloud<pcl::PointXY>::Ptr projections (new pcl::PointCloud<pcl::PointXY>);
        std::vector<bool> visibility;
        visibility.resize (mesh.tex_polygons[current_cam].size ());
        std::vector<UvIndex> indexes_uv_to_points;
        // for each current face

        //TODO change this
        pcl::PointXY nan_point;
        nan_point.x = std::numeric_limits<float>::quiet_NaN ();
        nan_point.y = std::numeric_limits<float>::quiet_NaN ();
        UvIndex u_null;
        u_null.idx_cloud = -1;
        u_null.idx_face = -1;

        int cpt_invisible=0;
        for (int idx_face = 0; idx_face <  static_cast<int> (mesh.tex_polygons[current_cam].size ()); ++idx_face)
        {
            //project each vertice, if one is out of view, stop
            pcl::PointXY uv_coord1;
            pcl::PointXY uv_coord2;
            pcl::PointXY uv_coord3;

            if (isFaceProjected (cameras[current_cam],
                                 (*camera_cloud)[mesh.tex_polygons[current_cam][idx_face].vertices[0]],
                                 (*camera_cloud)[mesh.tex_polygons[current_cam][idx_face].vertices[1]],
                                 (*camera_cloud)[mesh.tex_polygons[current_cam][idx_face].vertices[2]],
                                 uv_coord1,
                                 uv_coord2,
                                 uv_coord3))
            {
                // face is in the camera's FOV

                // add UV coordinates
                projections->points.push_back (uv_coord1);
                projections->points.push_back (uv_coord2);
                projections->points.push_back (uv_coord3);

                // remember corresponding face
                UvIndex u1, u2, u3;
                u1.idx_cloud = mesh.tex_polygons[current_cam][idx_face].vertices[0];
                u2.idx_cloud = mesh.tex_polygons[current_cam][idx_face].vertices[1];
                u3.idx_cloud = mesh.tex_polygons[current_cam][idx_face].vertices[2];
                u1.idx_face = idx_face; u2.idx_face = idx_face; u3.idx_face = idx_face;
                indexes_uv_to_points.push_back (u1);
                indexes_uv_to_points.push_back (u2);
                indexes_uv_to_points.push_back (u3);

                //keep track of visibility
                visibility[idx_face] = true;
            }
            else
            {
                projections->points.push_back (nan_point);
                projections->points.push_back (nan_point);
                projections->points.push_back (nan_point);
                indexes_uv_to_points.push_back (u_null);
                indexes_uv_to_points.push_back (u_null);
                indexes_uv_to_points.push_back (u_null);
                //keep track of visibility
                visibility[idx_face] = false;
                cpt_invisible++;
            }
        }

        // projections contains all UV points of the current faces
        // indexes_uv_to_points links a uv point to its point in the camera cloud
        // visibility contains tells if a face was in the camera FOV (false = skip)

        // TODO handle case were no face could be projected
        if (visibility.size () - cpt_invisible !=0)
        {
            //create kdtree
            pcl::KdTreeFLANN<pcl::PointXY> kdtree;
            kdtree.setInputCloud (projections);

            pcl::Indices idxNeighbors;
            std::vector<float> neighborsSquaredDistance;
            // af first (idx_pcam < current_cam), check if some of the faces attached to previous cameras occlude the current faces
            // then (idx_pcam == current_cam), check for self occlusions. At this stage, we skip faces that were already marked as occluded
            cpt_invisible = 0;
            for (int idx_pcam = 0 ; idx_pcam <= current_cam ; ++idx_pcam)
            {
                // project all faces
                for (int idx_face = 0; idx_face <  static_cast<int> (mesh.tex_polygons[idx_pcam].size ()); ++idx_face)
                {

                    if (idx_pcam == current_cam && !visibility[idx_face])
                    {
                        // we are now checking for self occlusions within the current faces
                        // the current face was already declared as occluded.
                        // therefore, it cannot occlude another face anymore => we skip it
                        continue;
                    }

                    // project each vertice, if one is out of view, stop
                    pcl::PointXY uv_coord1;
                    pcl::PointXY uv_coord2;
                    pcl::PointXY uv_coord3;

                    if (isFaceProjected (cameras[current_cam],
                                         (*camera_cloud)[mesh.tex_polygons[idx_pcam][idx_face].vertices[0]],
                                         (*camera_cloud)[mesh.tex_polygons[idx_pcam][idx_face].vertices[1]],
                                         (*camera_cloud)[mesh.tex_polygons[idx_pcam][idx_face].vertices[2]],
                                         uv_coord1,
                                         uv_coord2,
                                         uv_coord3))
                    {
                        // face is in the camera's FOV
                        //get its circumsribed circle
                        double radius;
                        pcl::PointXY center;
                        // getTriangleCircumcenterAndSize (uv_coord1, uv_coord2, uv_coord3, center, radius);
                        getTriangleCircumcscribedCircleCentroid(uv_coord1, uv_coord2, uv_coord3, center, radius); // this function yields faster results than getTriangleCircumcenterAndSize

                        // get points inside circ.circle
                        if (kdtree.radiusSearch (center, radius, idxNeighbors, neighborsSquaredDistance) > 0 )
                        {
                            // for each neighbor
                            for (const auto &idxNeighbor : idxNeighbors)
                            {
                                if (std::max ((*camera_cloud)[mesh.tex_polygons[idx_pcam][idx_face].vertices[0]].z,
                                              std::max ((*camera_cloud)[mesh.tex_polygons[idx_pcam][idx_face].vertices[1]].z,
                                                        (*camera_cloud)[mesh.tex_polygons[idx_pcam][idx_face].vertices[2]].z))
                                    < (*camera_cloud)[indexes_uv_to_points[idxNeighbor].idx_cloud].z)
                                {
                                    // neighbor is farther than all the face's points. Check if it falls into the triangle
                                    if (checkPointInsideTriangle(uv_coord1, uv_coord2, uv_coord3, (*projections)[idxNeighbor]))
                                    {
                                        // current neighbor is inside triangle and is closer => the corresponding face
                                        visibility[indexes_uv_to_points[idxNeighbor].idx_face] = false;
                                        cpt_invisible++;
                                        //TODO we could remove the projections of this face from the kd-tree cloud, but I found it slower, and I need the point to keep ordered to query UV coordinates later
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // now, visibility is true for each face that belongs to the current camera
        // if a face is not visible, we push it into the next one.

        if (static_cast<int> (mesh.tex_coordinates.size ()) <= current_cam)
        {
            std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > dummy_container;
            mesh.tex_coordinates.push_back (dummy_container);
        }
        mesh.tex_coordinates[current_cam].resize (3 * visibility.size ());

        std::vector<pcl::Vertices> occluded_faces;
        occluded_faces.resize (visibility.size ());
        std::vector<pcl::Vertices> visible_faces;
        visible_faces.resize (visibility.size ());

        int cpt_occluded_faces = 0;
        int cpt_visible_faces = 0;

        for (std::size_t idx_face = 0 ; idx_face < visibility.size () ; ++idx_face)
        {
            if (visibility[idx_face])
            {
                // face is visible by the current camera copy UV coordinates
                mesh.tex_coordinates[current_cam][cpt_visible_faces * 3](0) = (*projections)[idx_face*3].x;
                mesh.tex_coordinates[current_cam][cpt_visible_faces * 3](1) = (*projections)[idx_face*3].y;

                mesh.tex_coordinates[current_cam][cpt_visible_faces * 3 + 1](0) = (*projections)[idx_face*3 + 1].x;
                mesh.tex_coordinates[current_cam][cpt_visible_faces * 3 + 1](1) = (*projections)[idx_face*3 + 1].y;

                mesh.tex_coordinates[current_cam][cpt_visible_faces * 3 + 2](0) = (*projections)[idx_face*3 + 2].x;
                mesh.tex_coordinates[current_cam][cpt_visible_faces * 3 + 2](1) = (*projections)[idx_face*3 + 2].y;

                visible_faces[cpt_visible_faces] = mesh.tex_polygons[current_cam][idx_face];

                cpt_visible_faces++;
            }
            else
            {
                // face is occluded copy face into temp vector
                occluded_faces[cpt_occluded_faces] = mesh.tex_polygons[current_cam][idx_face];
                cpt_occluded_faces++;
            }
        }
        mesh.tex_coordinates[current_cam].resize (cpt_visible_faces*3);

        occluded_faces.resize (cpt_occluded_faces);
        mesh.tex_polygons.push_back (occluded_faces);

        visible_faces.resize (cpt_visible_faces);
        mesh.tex_polygons[current_cam].clear ();
        mesh.tex_polygons[current_cam] = visible_faces;
    }

    // we have been through all the cameras.
    // if any faces are left, they were not visible by any camera
    // we still need to produce uv coordinates for them

    if (mesh.tex_coordinates.size() <= cameras.size ())
    {
        std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > dummy_container;
        mesh.tex_coordinates.push_back(dummy_container);
    }


    for(std::size_t idx_face = 0 ; idx_face < mesh.tex_polygons[cameras.size()].size() ; ++idx_face)
    {
        Eigen::Vector2f UV1, UV2, UV3;
        UV1(0) = -1.0; UV1(1) = -1.0;
        UV2(0) = -1.0; UV2(1) = -1.0;
        UV3(0) = -1.0; UV3(1) = -1.0;
        mesh.tex_coordinates[cameras.size()].push_back(UV1);
        mesh.tex_coordinates[cameras.size()].push_back(UV2);
        mesh.tex_coordinates[cameras.size()].push_back(UV3);
    }

}

///////////////////////////////////////////////////////////////////////////////////////////////
inline void
urban_rec::Texturing_mapping::getTriangleCircumcenterAndSize(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, pcl::PointXY &circomcenter, double &radius)
{
    // we simplify the problem by translating the triangle's origin to its first point
    pcl::PointXY ptB, ptC;
    ptB.x = p2.x - p1.x; ptB.y = p2.y - p1.y; // B'=B-A
    ptC.x = p3.x - p1.x; ptC.y = p3.y - p1.y; // C'=C-A

    double D = 2.0*(ptB.x*ptC.y - ptB.y*ptC.x); // D'=2(B'x*C'y - B'y*C'x)

    // Safety check to avoid division by zero
    if(D == 0)
    {
        circomcenter.x = p1.x;
        circomcenter.y = p1.y;
    }
    else
    {
        // compute squares once
        double bx2 = ptB.x * ptB.x; // B'x^2
        double by2 = ptB.y * ptB.y; // B'y^2
        double cx2 = ptC.x * ptC.x; // C'x^2
        double cy2 = ptC.y * ptC.y; // C'y^2

        // compute circomcenter's coordinates (translate back to original coordinates)
        circomcenter.x = static_cast<float> (p1.x + (ptC.y*(bx2 + by2) - ptB.y*(cx2 + cy2)) / D);
        circomcenter.y = static_cast<float> (p1.y + (ptB.x*(cx2 + cy2) - ptC.x*(bx2 + by2)) / D);
    }

    radius = std::sqrt( (circomcenter.x - p1.x)*(circomcenter.x - p1.x)  + (circomcenter.y - p1.y)*(circomcenter.y - p1.y));//2.0* (p1.x*(p2.y - p3.y)  + p2.x*(p3.y - p1.y) + p3.x*(p1.y - p2.y));
}

///////////////////////////////////////////////////////////////////////////////////////////////
inline void
urban_rec::Texturing_mapping::getTriangleCircumcscribedCircleCentroid ( const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, pcl::PointXY &circumcenter, double &radius)
{
    // compute centroid's coordinates (translate back to original coordinates)
    circumcenter.x = static_cast<float> (p1.x + p2.x + p3.x ) / 3;
    circumcenter.y = static_cast<float> (p1.y + p2.y + p3.y ) / 3;
    double r1 = (circumcenter.x - p1.x) * (circumcenter.x - p1.x) + (circumcenter.y - p1.y) * (circumcenter.y - p1.y)  ;
    double r2 = (circumcenter.x - p2.x) * (circumcenter.x - p2.x) + (circumcenter.y - p2.y) * (circumcenter.y - p2.y)  ;
    double r3 = (circumcenter.x - p3.x) * (circumcenter.x - p3.x) + (circumcenter.y - p3.y) * (circumcenter.y - p3.y)  ;

    // radius
    radius = std::sqrt( std::max( r1, std::max( r2, r3) )) ;
}


///////////////////////////////////////////////////////////////////////////////////////////////
inline bool
urban_rec::Texturing_mapping::getPointUVCoordinates(const pcl::PointXYZ &pt, const Camera &cam, pcl::PointXY &UV_coordinates)
{
    if (pt.z > 0)
    {
        // compute image center and dimension
        double sizeX = cam.width;
        double sizeY = cam.height;
        double cx, cy;
        if (cam.center_w > 0)
            cx = cam.center_w;
        else
            cx = sizeX / 2.0;
        if (cam.center_h > 0)
            cy = cam.center_h;
        else
            cy = sizeY / 2.0;

        double focal_x, focal_y;
        if (cam.focal_length_w > 0)
            focal_x = cam.focal_length_w;
        else
            focal_x = cam.focal_length;
        if (cam.focal_length_h > 0)
            focal_y = cam.focal_length_h;
        else
            focal_y = cam.focal_length;

        // project point on camera's image plane
        UV_coordinates.x = static_cast<float> ((focal_x * (pt.x / pt.z) + cx) / sizeX); //horizontal
        UV_coordinates.y = 1.0f - static_cast<float> ((focal_y * (pt.y / pt.z) + cy) / sizeY); //vertical

        // point is visible!
        if (UV_coordinates.x >= 0.0 && UV_coordinates.x <= 1.0 && UV_coordinates.y >= 0.0 && UV_coordinates.y <= 1.0)
            return (true); // point was visible by the camera
    }

    // point is NOT visible by the camera
    UV_coordinates.x = -1.0f;
    UV_coordinates.y = -1.0f;
    return (false); // point was not visible by the camera
}

///////////////////////////////////////////////////////////////////////////////////////////////
inline bool
urban_rec::Texturing_mapping::checkPointInsideTriangle(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, const pcl::PointXY &pt)
{
    // Compute vectors
    Eigen::Vector2d v0, v1, v2;
    v0(0) = p3.x - p1.x; v0(1) = p3.y - p1.y; // v0= C - A
    v1(0) = p2.x - p1.x; v1(1) = p2.y - p1.y; // v1= B - A
    v2(0) = pt.x - p1.x; v2(1) = pt.y - p1.y; // v2= P - A

    // Compute dot products
    double dot00 = v0.dot(v0); // dot00 = dot(v0, v0)
    double dot01 = v0.dot(v1); // dot01 = dot(v0, v1)
    double dot02 = v0.dot(v2); // dot02 = dot(v0, v2)
    double dot11 = v1.dot(v1); // dot11 = dot(v1, v1)
    double dot12 = v1.dot(v2); // dot12 = dot(v1, v2)

    // Compute barycentric coordinates
    double invDenom = 1.0 / (dot00*dot11 - dot01*dot01);
    double u = (dot11*dot02 - dot01*dot12) * invDenom;
    double v = (dot00*dot12 - dot01*dot02) * invDenom;

    // Check if point is in triangle
    return ((u >= 0) && (v >= 0) && (u + v < 1));
}

///////////////////////////////////////////////////////////////////////////////////////////////
inline bool
urban_rec::Texturing_mapping::isFaceProjected (const Camera &camera, const pcl::PointXYZ &p1, const pcl::PointXYZ &p2, const pcl::PointXYZ &p3, pcl::PointXY &proj1, pcl::PointXY &proj2, pcl::PointXY &proj3)
{
    return (getPointUVCoordinates(p1, camera, proj1)
            &&
            getPointUVCoordinates(p2, camera, proj2)
            &&
            getPointUVCoordinates(p3, camera, proj3)
    );
}


tuple<pcl::TextureMesh, pcl::texture_mapping::CameraVector> urban_rec::Texturing_mapping::textureMesh(vector <string> argv) {
    pcl::PolygonMesh triangles;
    if (input_polygon_mesh != nullptr) {
        triangles = *input_polygon_mesh;
    } else {
        pcl::io::loadPolygonFilePLY(argv[0], triangles);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new PointCloud <PointXYZ>);
    pcl::fromPCLPointCloud2(triangles.cloud, *cloud);

    // Create the texturemesh object that will contain our UV-mapped mesh
    pcl::TextureMesh mesh;
    mesh.cloud = triangles.cloud;
    vector <pcl::Vertices> polygon_1;

    // Push faces into the texturemesh object
    polygon_1.resize(triangles.polygons.size());
    for (size_t i = 0; i < triangles.polygons.size(); ++i) {
        polygon_1[i] = triangles.polygons[i];
    }
    mesh.tex_polygons.push_back(polygon_1);

    // Load textures and cameras poses and intrinsics
    pcl::texture_mapping::CameraVector my_cams;

    const boost::filesystem::path base_dir(argv[2]);
    std::string extension(".txt");
    std::vector <boost::filesystem::path> filenames;
    try {
        for (boost::filesystem::directory_iterator it(base_dir);
             it != boost::filesystem::directory_iterator(); ++it) {
            if (boost::filesystem::is_regular_file(it->status()) &&
                boost::filesystem::extension(it->path()) == extension) {
                filenames.push_back(it->path());
            }
        }
    } catch (const boost::filesystem::filesystem_error &e) {
        cerr << e.what() << endl;
    }
    std::sort(filenames.begin(), filenames.end());


    for (int i = 0; i < filenames.size(); ++i) {
        std::cout << filenames[i].string() << std::endl;
        pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
        readCamPoseFile(filenames[i].string(), cam);
        cam.texture_file = filenames[i].stem().string() + ".jpg"; //".png"
        my_cams.push_back(cam);
    }

    // Create materials for each texture (and one extra for occluded faces)
    mesh.tex_materials.resize(my_cams.size() + 1);
    for (int i = 0; i <= my_cams.size(); ++i) {
        pcl::TexMaterial mesh_material;
        mesh_material.tex_Ka.r = 0.2f;
        mesh_material.tex_Ka.g = 0.2f;
        mesh_material.tex_Ka.b = 0.2f;

        mesh_material.tex_Kd.r = 0.8f;
        mesh_material.tex_Kd.g = 0.8f;
        mesh_material.tex_Kd.b = 0.8f;

        mesh_material.tex_Ks.r = 1.0f;
        mesh_material.tex_Ks.g = 1.0f;
        mesh_material.tex_Ks.b = 1.0f;

        mesh_material.tex_d = 1.0f;
        mesh_material.tex_Ns = 75.0f;
        mesh_material.tex_illum = 2;

        std::stringstream tex_name;
        tex_name << "material_" << i;
        tex_name >> mesh_material.tex_name;

        if (i < my_cams.size()) {
            mesh_material.tex_file = my_cams[i].texture_file;
        } else {
            mesh_material.tex_file = "occluded.jpg";
        }
        mesh.tex_materials[i] = mesh_material;
    }

    // Sort faces
//    pcl::TextureMapping <pcl::PointXYZ> tm;  // TextureMapping object that will perform the sort
    textureMeshwithMultipleCameras(mesh, my_cams);

    // Compute normals for the mesh
    pcl::NormalEstimation <pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    // Concatenate XYZ and normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud <pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    pcl::toPCLPointCloud2(*cloud_with_normals, mesh.cloud);
//    pcl::io::saveOBJFile(argv[1], mesh, 5);
    saveOBJFile(argv[1], mesh, 5);

    return make_tuple(mesh, my_cams);
}

vector<pcl::TextureMesh> urban_rec::Texturing_mapping::textureMeshes(vector <string> argv) {
    pcl::PolygonMesh triangles;
    if (input_polygon_mesh != nullptr) {
        triangles = *input_polygon_mesh;
    } else {
        pcl::io::loadPolygonFilePLY(argv[0], triangles);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new PointCloud <PointXYZ>);
    pcl::fromPCLPointCloud2(triangles.cloud, *cloud);

    // Read images with txt files cam's parameters
    const boost::filesystem::path base_dir(argv[2]);
    std::string extension(".txt");
    std::vector <boost::filesystem::path> filenames;
    try {
        for (boost::filesystem::directory_iterator it(base_dir);
             it != boost::filesystem::directory_iterator(); ++it) {
            if (boost::filesystem::is_regular_file(it->status()) &&
                boost::filesystem::extension(it->path()) == extension) {
                filenames.push_back(it->path());
            }
        }
    } catch (const boost::filesystem::filesystem_error &e) {
        cerr << e.what() << endl;
    }
    std::sort(filenames.begin(), filenames.end());

    // Create the texturemesh object that will contain our UV-mapped mesh
    vector<pcl::TextureMesh> mesh(filenames.size());
    for (int i = 0; i < filenames.size(); i++) {
        mesh[i].cloud = triangles.cloud;
    }

    vector <pcl::Vertices> polygon_1;
    // Push faces into the texturemesh object
    for (int i = 0; i < filenames.size(); i++) {
        polygon_1.resize(triangles.polygons.size());
        for (size_t i = 0; i < triangles.polygons.size(); ++i) {
            polygon_1[i] = triangles.polygons[i];
        }
        mesh[i].tex_polygons.push_back(polygon_1);
    }

    // Load textures and cameras poses and intrinsics
    vector<pcl::texture_mapping::CameraVector> my_cams(filenames.size());

    for (int i = 0; i < filenames.size(); ++i) {
        std::cout << filenames[i].string() << std::endl;
        pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
        readCamPoseFile(filenames[i].string(), cam);
        cam.texture_file = filenames[i].stem().string() + ".jpg"; //".png"
        my_cams[i].push_back(cam);
    }

    // Create materials for each texture (and one extra for occluded faces)
    for (int i = 0; i < filenames.size(); ++i) {
        mesh[i].tex_materials.resize(2);
    }
    for (int i = 0; i <= my_cams.size(); ++i) {
        pcl::TexMaterial mesh_material;
        mesh_material.tex_Ka.r = 0.2f;
        mesh_material.tex_Ka.g = 0.2f;
        mesh_material.tex_Ka.b = 0.2f;

        mesh_material.tex_Kd.r = 0.8f;
        mesh_material.tex_Kd.g = 0.8f;
        mesh_material.tex_Kd.b = 0.8f;

        mesh_material.tex_Ks.r = 1.0f;
        mesh_material.tex_Ks.g = 1.0f;
        mesh_material.tex_Ks.b = 1.0f;

        mesh_material.tex_d = 1.0f;
        mesh_material.tex_Ns = 75.0f;
        mesh_material.tex_illum = 2;

        std::stringstream tex_name;
        tex_name << "material_" << i;
        tex_name >> mesh_material.tex_name;

        if (i < my_cams.size()) {
            mesh_material.tex_file = my_cams[i][0].texture_file;
            mesh[i].tex_materials[0] = mesh_material;
        } else {
            mesh_material.tex_file = "occluded.jpg";
            for (int j = 0; j < filenames.size(); ++j) {
                mesh[j].tex_materials[1] = mesh_material;
            }
        }
    }

    // Sort faces
    for (int i = 0; i < filenames.size(); ++i) {
        //    pcl::TextureMapping <pcl::PointXYZ> tm;  // TextureMapping object that will perform the sort
        textureMeshwithMultipleCameras(mesh[i], my_cams[i]);
        cout << "Ended..." << endl;
    }

    // Compute normals for the mesh
    pcl::NormalEstimation <pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    // Concatenate XYZ and normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud <pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // Creating pathes for obj files
    int dot_index = path_utils::getIndexBeforeChar(argv[1], '.');
    string obj_subpath1 = argv[1].substr(0, dot_index);
    string obj_subpath2 = argv[1].substr(dot_index, argv[1].size());

    for (int i = 0; i < filenames.size(); ++i) {
        pcl::toPCLPointCloud2(*cloud_with_normals, mesh[i].cloud);
        std::ostringstream oss_dest;
        oss_dest << obj_subpath1 << "_" << i << obj_subpath2;
        saveOBJFile(oss_dest.str(), mesh[i], 5);
    }

    return mesh;
}