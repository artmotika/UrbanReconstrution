#include "BestFittingTexturingQuality.h"

using namespace std;
using namespace pcl;

void BestFittingTexturingQuality::setNumberCams(int num) {
    number_cams = num;
}

int BestFittingTexturingQuality::getNumberCams() {
    return number_cams;
}

void BestFittingTexturingQuality::setInputPolygonMesh(pcl::PolygonMesh &mesh) {
    triangles = mesh;
}

void BestFittingTexturingQuality::setInputTextureMeshes(vector<pcl::TextureMesh> &meshes) {
    input_meshes = meshes;
}

pcl::TextureMesh BestFittingTexturingQuality::fit(vector <string> argv) {
    urban_rec::TexturingMapping texturing_mapping = urban_rec::TexturingMapping(2000, 2000);
    // Create the texturemesh object that will contain our UV-mapped mesh
    pcl::TextureMesh mesh;
    mesh.cloud = triangles.cloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new PointCloud <PointXYZ>);
    pcl::fromPCLPointCloud2(triangles.cloud, *cloud);

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
        texturing_mapping.readCamPoseFile(filenames[i].string(), cam);
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

    // Create image masks
    vector <cv::Mat> masks;
    // Сохраняем текстуры маски в матрицы из opencv
    std::ostringstream oss_masks;
    oss_masks << argv[2] << "masks/mask_Xminus.jpg";
    string masks_path = oss_masks.str();
    cout << masks_path << endl;
    cv::Mat imageMask = cv::imread(masks_path, cv::IMREAD_COLOR);
    masks.push_back(imageMask);
    oss_masks.str("");
    oss_masks << argv[2] << "masks/mask_Xplus.jpg";
    masks_path = oss_masks.str();
    cout << masks_path << endl;
    imageMask = cv::imread(masks_path, cv::IMREAD_COLOR);
    masks.push_back(imageMask);
    oss_masks.str("");
    oss_masks << argv[2] << "masks/mask_Yminus.jpg";
    masks_path = oss_masks.str();
    cout << masks_path << endl;
    imageMask = cv::imread(masks_path, cv::IMREAD_COLOR);
    masks.push_back(imageMask);
    oss_masks.str("");
    oss_masks << argv[2] << "masks/mask_Yplus.jpg";
    masks_path = oss_masks.str();
    cout << masks_path << endl;
    imageMask = cv::imread(masks_path, cv::IMREAD_COLOR);
    masks.push_back(imageMask);
    oss_masks.str("");
    oss_masks << argv[2] << "masks/mask_Zminus.jpg";
    masks_path = oss_masks.str();
    cout << masks_path << endl;
    imageMask = cv::imread(masks_path, cv::IMREAD_COLOR);
    masks.push_back(imageMask);

    vector <vector <int>> meshFaceIndexMapInputMeshesFullToPart(number_cams);
    vector <vector <int>> meshFaceIndexMapInputMeshesPartToFull(number_cams);
    for (int current_cam = 0; current_cam < number_cams; current_cam++) {
        FaceIndexMaps faceIndexMaps = FaceIndexMaps(triangles.polygons,
                                                    input_meshes[current_cam].tex_polygons[0],
                                                    cloud->size());
        faceIndexMaps.getFaceIndexMaps();
        meshFaceIndexMapInputMeshesFullToPart[current_cam] = faceIndexMaps.faceIndexMapFullToPart;
        meshFaceIndexMapInputMeshesPartToFull[current_cam] = faceIndexMaps.faceIndexMapPartToFull;
    }

    for (int current_cam = 0; current_cam < number_cams + 1; current_cam++) {
        vector  <Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > dummy_container;
        mesh.tex_coordinates.push_back (dummy_container);
        vector <pcl::Vertices> polygon_dummy;
        mesh.tex_polygons.push_back(polygon_dummy);
    }

    for (int global_face_idx = 0; global_face_idx < triangles.polygons.size(); global_face_idx++) {
        double max_area = 0.0;
        int face_idx = -1;
        int cam_idx = -1;
        for (int current_cam = 0; current_cam < number_cams; current_cam++) {
            int local_face_idx = meshFaceIndexMapInputMeshesFullToPart[current_cam][global_face_idx];
            if (local_face_idx == -1) continue;
            pcl::PointXY p0 = PointXY(input_meshes[current_cam].tex_coordinates[0][local_face_idx * 3](0),
                                    input_meshes[current_cam].tex_coordinates[0][local_face_idx * 3](1));
            pcl::PointXY p1 = PointXY(input_meshes[current_cam].tex_coordinates[0][local_face_idx * 3 + 1](0),
                                    input_meshes[current_cam].tex_coordinates[0][local_face_idx * 3 + 1](1));
            pcl::PointXY p2 = PointXY(input_meshes[current_cam].tex_coordinates[0][local_face_idx * 3 + 2](0),
                                    input_meshes[current_cam].tex_coordinates[0][local_face_idx * 3 + 2](1));
            int mask_idx = current_cam % 6;
            if (mask_idx != 5 && texturing_mapping.isFaceOnMask(p0, p1, p2, masks[mask_idx])) {
                continue;
            }

            double area = Geometry_pcl::triangle_area(p0, p1, p2);
            if (area > max_area) {
                max_area = area;
                face_idx = local_face_idx;
                cam_idx = current_cam;
            }
        }
        if (face_idx != -1) {
            Eigen::Vector2f tex_coords1 = input_meshes[cam_idx].tex_coordinates[0][face_idx * 3];
            Eigen::Vector2f tex_coords2 = input_meshes[cam_idx].tex_coordinates[0][face_idx * 3 + 1];
            Eigen::Vector2f tex_coords3 = input_meshes[cam_idx].tex_coordinates[0][face_idx * 3 + 2];
            mesh.tex_coordinates[cam_idx].push_back(tex_coords1);
            mesh.tex_coordinates[cam_idx].push_back(tex_coords2);
            mesh.tex_coordinates[cam_idx].push_back(tex_coords3);
            pcl::Vertices polygon = triangles.polygons[global_face_idx];
            mesh.tex_polygons[cam_idx].push_back(polygon);
        } else {
            pcl::Vertices polygon = triangles.polygons[global_face_idx];
            mesh.tex_polygons[number_cams].push_back(polygon);
        }
    }

    for (std::size_t face_idx = 0 ; face_idx < mesh.tex_polygons[number_cams].size() ; ++face_idx) {
        Eigen::Vector2f UV1, UV2, UV3;
        UV1(0) = -1.0; UV1(1) = -1.0;
        UV2(0) = -1.0; UV2(1) = -1.0;
        UV3(0) = -1.0; UV3(1) = -1.0;
        mesh.tex_coordinates[number_cams].push_back(UV1);
        mesh.tex_coordinates[number_cams].push_back(UV2);
        mesh.tex_coordinates[number_cams].push_back(UV3);
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

    pcl::toPCLPointCloud2(*cloud_with_normals, mesh.cloud);
    texturing_mapping.saveOBJFile(argv[1], mesh, 5); // example_mini5

    return mesh;
}