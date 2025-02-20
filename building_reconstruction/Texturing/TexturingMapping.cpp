#include "TexturingMapping.h"
#include "IO/Image_txt_reading.h"

using namespace urban_rec;
using namespace std;
using namespace pcl;

int urban_rec::TexturingMapping::saveOBJFile(const std::string &file_name,
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
            if ((tex_mesh.cloud.fields[d].name == "normal_x" ||
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

void urban_rec::TexturingMapping::setTextureWidthAndHeight(int width, int height) {
    texture_width = width;
    texture_height = height;
}

void urban_rec::TexturingMapping::setInputPolygonMesh(pcl::PolygonMesh &polygon_mesh) {
    input_polygon_mesh = std::make_shared<pcl::PolygonMesh>(polygon_mesh);
}

pcl::PolygonMesh urban_rec::TexturingMapping::getInputPolygonMesh() {
    return *input_polygon_mesh;
}

bool
urban_rec::TexturingMapping::getPointUVCoords(const PointXYZ &pt, const pcl::TextureMapping<pcl::PointXYZ>::Camera &cam,
                                              PointXY &UV_coordinates) {
    // if the point is in front of the camera
    if (pt.z > 0) {
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
    return (false); // point was not visible by the camera
}

bool urban_rec::TexturingMapping::readCamPoseFile(std::string filename,
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

bool urban_rec::TexturingMapping::isFaceOnMask(pcl::PointXY &proj1, pcl::PointXY &proj2, pcl::PointXY &proj3,
                                               const cv::Mat &mask) {
    int x0 = proj1.x * float(texture_width);
    int y0 = float(texture_height) - proj1.y * float(texture_height);
    int x1 = proj2.x * float(texture_width);
    int y1 = float(texture_height) - proj2.y * float(texture_height);
    int x2 = proj3.x * float(texture_width);
    int y2 = float(texture_height) - proj3.y * float(texture_height);
    cv::Vec3b black_pixel = cv::Vec3b(0, 0, 0);
    if (mask.at<cv::Vec3b>(y0, x0) == black_pixel
        || mask.at<cv::Vec3b>(y1, x1) == black_pixel
        || mask.at<cv::Vec3b>(y2, x2) == black_pixel) {
        return true;
    }
    return false;
}

bool urban_rec::TexturingMapping::isFaceProjectedAngleMore(const pcl::PointXYZ &p1,
                                                           const pcl::PointXYZ &p2,
                                                           const pcl::PointXYZ &p3,
                                                           const pcl::PointXYZ &camPose,
                                                           double angle,
                                                           double max_dist) {
    pcl::PointXYZ center = Geometry_pcl::getTriangleCenterOfMass(p1, p2, p3);
    if (Geometry_pcl::euclidean_dist_between_two_points(center, camPose) < max_dist) return true;
    Eigen::Vector3d vectorAB(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    Eigen::Vector3d vectorAC(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
    Eigen::Vector3d vectorNormal = vectorAB.cross(vectorAC);
    Eigen::Vector3d vectorCamPoseToCenter(center.x - camPose.x, center.y - camPose.y, center.z - camPose.z);
    double scalar_product = vectorNormal.dot(vectorCamPoseToCenter);
    double calculated_angle =
            std::acos(scalar_product / (vectorNormal.norm() * vectorCamPoseToCenter.norm())) * (180 / M_PI);
    if (calculated_angle > 90.0) calculated_angle = 180.0 - calculated_angle;
    if (calculated_angle > angle) {
        return true;
    }
    return false;
}


tuple <pcl::TextureMesh, pcl::texture_mapping::CameraVector>
urban_rec::TexturingMapping::textureMesh(vector <string> argv) {
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
                (it->path().extension().string() == extension)) {
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
        mesh_material.tex_Ka.r = KA_R;
        mesh_material.tex_Ka.g = KA_G;
        mesh_material.tex_Ka.b = KA_B;

        mesh_material.tex_Kd.r = KD_R;
        mesh_material.tex_Kd.g = KD_G;
        mesh_material.tex_Kd.b = KD_B;

        mesh_material.tex_Ks.r = KS_R;
        mesh_material.tex_Ks.g = KS_G;
        mesh_material.tex_Ks.b = KS_B;

        mesh_material.tex_d = D;
        mesh_material.tex_Ns = NS;
        mesh_material.tex_illum = ILLUM;

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
    pcl::TextureMapping <pcl::PointXYZ> tm;  // TextureMapping object that will perform the sort
    tm.textureMeshwithMultipleCameras(mesh, my_cams);

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

vector <pcl::TextureMesh> urban_rec::TexturingMapping::textureMeshes(vector <string> argv) {
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
                (it->path().extension().string() == extension)) {
                filenames.push_back(it->path());
            }
        }
    } catch (const boost::filesystem::filesystem_error &e) {
        cerr << e.what() << endl;
    }
    std::sort(filenames.begin(), filenames.end());

    // Create the texturemesh object that will contain our UV-mapped mesh
    vector <pcl::TextureMesh> mesh(filenames.size());
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
    vector <pcl::texture_mapping::CameraVector> my_cams(filenames.size());

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
        mesh_material.tex_Ka.r = KA_R;
        mesh_material.tex_Ka.g = KA_G;
        mesh_material.tex_Ka.b = KA_B;

        mesh_material.tex_Kd.r = KD_R;
        mesh_material.tex_Kd.g = KD_G;
        mesh_material.tex_Kd.b = KD_B;

        mesh_material.tex_Ks.r = KS_R;
        mesh_material.tex_Ks.g = KS_G;
        mesh_material.tex_Ks.b = KS_B;

        mesh_material.tex_d = D;
        mesh_material.tex_Ns = NS;
        mesh_material.tex_illum = ILLUM;

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
        pcl::TextureMapping <pcl::PointXYZ> tm;  // TextureMapping object that will perform the sort
        tm.textureMeshwithMultipleCameras(mesh[i], my_cams[i]);
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

//    for (int i = 0; i < filenames.size(); ++i) {
//        pcl::toPCLPointCloud2(*cloud_with_normals, mesh[i].cloud);
//        std::ostringstream oss_dest;
//        oss_dest << obj_subpath1 << "_" << i << obj_subpath2;
//        saveOBJFile(oss_dest.str(), mesh[i], 5);
//    }

    return mesh;
}