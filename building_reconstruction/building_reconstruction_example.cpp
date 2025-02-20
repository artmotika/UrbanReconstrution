#include "Building_reconstruction.h"
#include "Algo_reconstruction.h"

using namespace pcl;
using namespace urban_rec;

int main() {
    Building_reconstruction building_reconstruction;
    building_reconstruction.setPoissonDepth(10);
    building_reconstruction.setFilterRadius(0.4);
    building_reconstruction.setInputFile(
            "../data/Construction_home_sample_normals.pcd"); //Construction_corner_oriented_normals_not_sample.pcd//Construction_corner_sample_oriented_normals.pcd
    building_reconstruction.setInputFileSurfaces(
            "../data/Construction_home_sample_normals_plane.ply"); //region_growing_sample.ply
    building_reconstruction.setOutputFile("../data/building_reconstruction2.ply");

    PolygonMesh surfaces_mesh;
    Io_pcl::loadCloud("../data/Construction_home_sample_normals_plane.ply", surfaces_mesh);

    PCLPointCloud2::Ptr ideal_cloud(new PCLPointCloud2);
    PCLPointCloud2::Ptr poisson_cloud(new PCLPointCloud2);
    PCLPointCloud2::Ptr pivot_cloud(new PCLPointCloud2);
    PCLPointCloud2::Ptr alpha_shape_cloud(new PCLPointCloud2);
    PCLPointCloud2::Ptr scale_space_cloud(new PCLPointCloud2);
    PCLPointCloud2::Ptr delaunay_cloud(new PCLPointCloud2);
    PCLPointCloud2::Ptr greedy_cloud(new PCLPointCloud2);

    PolygonMesh greedy_mesh;
    PolygonMesh delaunay_mesh;
    PolygonMesh scale_space_mesh;
    PolygonMesh alpha_shape_mesh;
    PolygonMesh pivot_mesh;
    PolygonMesh poisson_plane_mesh;
    PolygonMesh poisson_mesh;
    PolygonMesh ideal_mesh;
    int poisson_depth = 10;
    int solver_divide = 8;
    int iso_divide = 8;
    float poisson_point_weight = 4.0f;
    double greedy_radius = 2.5; // (почти тоже самое что и) максимальная длина ребра
    double greedy_mu = 3.0; //

    // ideal
    Io_pcl::loadCloud("../data/Construction_home_sample_normals_ideal_point_cloud.pcd", *ideal_cloud);
    algo_rec::computePoisson(ideal_cloud, ideal_mesh, poisson_depth, solver_divide, iso_divide, poisson_point_weight);
    ideal_mesh = building_reconstruction.filterMeshPoissonByPointsNoExtra(
            ideal_mesh,
            std::make_shared<PCLPointCloud2>(surfaces_mesh.cloud), 2.4);
    PointCloud<PointXYZ>::Ptr ideal_points = building_reconstruction.filterPointsByMesh(
            ideal_mesh);
    Io_pcl::saveCloud("../data/ideal.ply", ideal_mesh);
    Io_pcl::saveCloud("../data/ideal_points.ply", *ideal_points);
    building_reconstruction.upsampleMesh(ideal_points, ideal_mesh, 0.004, 0.1);
    Io_pcl::saveCloud("../data/ideal_points_upsample.ply", *ideal_points);

    // poisson_repeatable
    poisson_plane_mesh = building_reconstruction.reconstruct();
    poisson_plane_mesh = building_reconstruction.filterMeshPoissonByPointsNoExtra(
            poisson_plane_mesh,
            std::make_shared<PCLPointCloud2>(surfaces_mesh.cloud), 2.4);
    PointCloud<PointXYZ>::Ptr poisson_plane_points = building_reconstruction.filterPointsByMesh(
            poisson_plane_mesh);
    Io_pcl::saveCloud("../data/poisson_plane.ply", poisson_plane_mesh);
    Io_pcl::saveCloud("../data/poisson_plane_points.ply", *poisson_plane_points); //*poisson_plane_points
    building_reconstruction.upsampleMesh(poisson_plane_points, poisson_plane_mesh, 0.004, 0.1);
    Io_pcl::saveCloud("../data/poisson_plane_points_upsample.ply", *poisson_plane_points);


    // poisson_repeatable_not_plane
    Io_pcl::loadCloud("../data/Construction_home_sample_normals.pcd", *poisson_cloud);
    algo_rec::computePoisson(poisson_cloud, poisson_mesh, poisson_depth,
                             solver_divide, iso_divide, poisson_point_weight);
    poisson_mesh = building_reconstruction.filterMeshPoissonByPointsNoExtra(
            poisson_mesh,
            std::make_shared<PCLPointCloud2>(surfaces_mesh.cloud), 1.2);
    PointCloud<PointXYZ>::Ptr poisson_points = building_reconstruction.filterPointsByMesh(
            poisson_mesh);
    Io_pcl::saveCloud("../data/poisson.ply", poisson_mesh);
    Io_pcl::saveCloud("../data/poisson_points.ply", *poisson_points);
    building_reconstruction.upsampleMesh(poisson_points,
                                         poisson_mesh, 0.04, 0.2);
    Io_pcl::saveCloud("../data/poisson_points_upsample.ply", *poisson_points);

    // ball_pivot_repeatable_not_plane
    Io_pcl::loadCloud("../data/ball_pivot3_mesh.ply", pivot_mesh);
    pivot_mesh = building_reconstruction.filterMeshByPoints(
            pivot_mesh,
            std::make_shared<PCLPointCloud2>(surfaces_mesh.cloud), 1.2);
    PointCloud<PointXYZ>::Ptr pivot_points = building_reconstruction.filterPointsByMesh(
            pivot_mesh);
    Io_pcl::saveCloud("../data/pivot.ply", pivot_mesh);
    Io_pcl::saveCloud("../data/pivot_points.ply", *pivot_points);
    building_reconstruction.upsampleMesh(pivot_points,
                                         pivot_mesh, 0.04, 0.2);
    Io_pcl::saveCloud("../data/pivot_points_upsample.ply", *pivot_points);

    // alpha_shape
    Io_pcl::loadCloud("../data/alpha_shape2_mesh.ply", alpha_shape_mesh);
    alpha_shape_mesh = building_reconstruction.filterMeshByPoints(
            alpha_shape_mesh,
            std::make_shared<PCLPointCloud2>(surfaces_mesh.cloud), 1.2);
    PointCloud<PointXYZ>::Ptr alpha_shape_points = building_reconstruction.filterPointsByMesh(
            alpha_shape_mesh);
    Io_pcl::saveCloud("../data/alpha_shape.ply", alpha_shape_mesh);
    Io_pcl::saveCloud("../data/alpha_shape_points.ply", *alpha_shape_points);
    building_reconstruction.upsampleMesh(alpha_shape_points,
                                         alpha_shape_mesh, 0.04, 0.2);
    Io_pcl::saveCloud("../data/alpha_shape_points_upsample.ply", *alpha_shape_points);

    // scale_space
    Io_pcl::loadCloud("../data/scale_space_mesh.ply", scale_space_mesh);
    scale_space_mesh = building_reconstruction.filterMeshByPoints(
            scale_space_mesh,
            std::make_shared<PCLPointCloud2>(surfaces_mesh.cloud), 1.2);
    PointCloud<PointXYZ>::Ptr scale_space_points = building_reconstruction.filterPointsByMesh(
            scale_space_mesh);
    Io_pcl::saveCloud("../data/scale_space.ply", scale_space_mesh);
    Io_pcl::saveCloud("../data/scale_space_points.ply", *scale_space_points);
    building_reconstruction.upsampleMesh(scale_space_points,
                                         scale_space_mesh, 0.04, 0.2);
    Io_pcl::saveCloud("../data/scale_space_points_upsample.ply", *scale_space_points);

    // delaunay
    Io_pcl::loadCloud("../data/delaunay_mesh.ply", delaunay_mesh);
    delaunay_mesh = building_reconstruction.filterMeshByPoints(
            delaunay_mesh,
            std::make_shared<PCLPointCloud2>(surfaces_mesh.cloud), 1.2);
    PointCloud<PointXYZ>::Ptr delaunay_points = building_reconstruction.filterPointsByMesh(
            delaunay_mesh);
    Io_pcl::saveCloud("../data/delaunay.ply", delaunay_mesh);
    Io_pcl::saveCloud("../data/delaunay_points.ply", *delaunay_points);
    building_reconstruction.upsampleMesh(delaunay_points,
                                         delaunay_mesh, 0.04, 0.2);
    Io_pcl::saveCloud("../data/delaunay_points_upsample.ply", *delaunay_points);

    // greedy_triangulation
    Io_pcl::loadCloud("../data/Construction_home_sample_normals.pcd", *greedy_cloud);
    algo_rec::computeGreedyTriangulation(greedy_cloud, greedy_mesh, greedy_mu, greedy_radius);
    Io_pcl::saveCloud("../data/greedy_mesh.ply", greedy_mesh);
    greedy_mesh = building_reconstruction.filterMeshByPoints(
            greedy_mesh,
            std::make_shared<PCLPointCloud2>(surfaces_mesh.cloud), 1.2);
    PointCloud<PointXYZ>::Ptr greedy_points = building_reconstruction.filterPointsByMesh(
            greedy_mesh);
    Io_pcl::saveCloud("../data/greedy.ply", greedy_mesh);
    Io_pcl::saveCloud("../data/greedy_points.ply", *greedy_points);
    building_reconstruction.upsampleMesh(greedy_points,
                                         greedy_mesh, 0.04, 0.2);
    Io_pcl::saveCloud("../data/greedy_points_upsample.ply", *greedy_points);


    // metrics
    double repeat_mistake = 0.3; //0.2
    double hole_mistake = 0.15; //0.15
    Io_pcl::loadCloudPLY("../data/ideal_points_partnew.ply", *ideal_cloud);
    PCLPointCloud2::Ptr ideal_cloud_upsample(new PCLPointCloud2);
    Io_pcl::loadCloudPLY("../data/ideal_points_upsample_partnew.ply", *ideal_cloud_upsample);

    std::pair<unsigned long, double> metric_pair;

    // poisson_plane
    PCLPointCloud2::Ptr poisson_plane_cloud(new PCLPointCloud2);
    Io_pcl::loadCloudPLY("../data/poisson_plane_points_part.ply", *poisson_plane_cloud);
    metric_pair = building_reconstruction.calculateRepeatabilityMetric(ideal_cloud_upsample,
                                                                       poisson_plane_cloud,
                                                                       repeat_mistake,
                                                                       "../data/poisson_plane_not_repeat_part.ply");
    std::cout << std::endl
              << "ideal_points_upsample and poisson_plane_points repeatability_metric with max_mistake = "
              << repeat_mistake << ": "
              << "count: " << metric_pair.first
              << " value: " << metric_pair.second
              << std::endl;
    PCLPointCloud2::Ptr poisson_plane_cloud_upsample(new PCLPointCloud2);
    Io_pcl::loadCloudPLY("../data/poisson_plane_points_upsample_part.ply", *poisson_plane_cloud_upsample);
    metric_pair = building_reconstruction.calculateHoleMetric(ideal_cloud_upsample,
                                                              poisson_plane_cloud_upsample,
                                                              hole_mistake,
                                                              "../data/poisson_plane_hole_part.ply");
    std::cout << std::endl
              << "ideal_points_upsample and poisson_plane hole_metric with max_mistake = " << hole_mistake << ": "
              << "count: " << metric_pair.first
              << " value: " << metric_pair.second
              << std::endl;


    // poisson
    Io_pcl::loadCloudPLY("../data/poisson_points_part.ply", *poisson_cloud);
    metric_pair = building_reconstruction.calculateRepeatabilityMetric(ideal_cloud_upsample,
                                                                       poisson_cloud,
                                                                       repeat_mistake,
                                                                       "../data/poisson_not_repeat_part.ply");
    std::cout << std::endl
              << "ideal_points_upsample and poisson_points repeatability_metric with max_mistake = " << repeat_mistake
              << ": "
              << "count: " << metric_pair.first
              << " value: " << metric_pair.second
              << std::endl;
    PCLPointCloud2::Ptr poisson_cloud_upsample(new PCLPointCloud2);
    Io_pcl::loadCloudPLY("../data/poisson_points_upsample_part.ply", *poisson_cloud_upsample);
    metric_pair = building_reconstruction.calculateHoleMetric(ideal_cloud_upsample,
                                                              poisson_cloud_upsample,
                                                              hole_mistake,
                                                              "../data/poisson_hole_part.ply");
    std::cout << std::endl
              << "ideal_points_upsample and poisson hole_metric with max_mistake = " << hole_mistake << ": "
              << "count: " << metric_pair.first
              << " value: " << metric_pair.second
              << std::endl;


    // pivot
    Io_pcl::loadCloudPLY("../data/pivot_points_part.ply", *pivot_cloud);
    metric_pair = building_reconstruction.calculateRepeatabilityMetric(ideal_cloud_upsample,
                                                                       pivot_cloud,
                                                                       repeat_mistake,
                                                                       "../data/pivot_not_repeat_part.ply");
    std::cout << std::endl
              << "ideal_points_upsample and pivot_points repeatability_metric with max_mistake = " << repeat_mistake
              << ": "
              << "count: " << metric_pair.first
              << " value: " << metric_pair.second

              << std::endl;
    PCLPointCloud2::Ptr pivot_cloud_upsample(new PCLPointCloud2);
    Io_pcl::loadCloudPLY("../data/pivot_points_upsample_part.ply", *pivot_cloud_upsample);
    metric_pair = building_reconstruction.calculateHoleMetric(ideal_cloud_upsample,
                                                              pivot_cloud_upsample,
                                                              hole_mistake,
                                                              "../data/pivot_hole_part.ply");
    std::cout << std::endl
              << "ideal_points_upsample and pivot hole_metric with max_mistake = " << hole_mistake << ": "
              << "count: " << metric_pair.first
              << " value: " << metric_pair.second
              << std::endl;


    // alpha_shape
    Io_pcl::loadCloudPLY("../data/alpha_shape_points_part.ply", *alpha_shape_cloud);
    metric_pair = building_reconstruction.calculateRepeatabilityMetric(ideal_cloud_upsample,
                                                                       alpha_shape_cloud,
                                                                       repeat_mistake,
                                                                       "../data/alpha_shape_not_repeat_part.ply");
    std::cout << std::endl
              << "ideal_points_upsample and alpha_shape_points repeatability_metric with max_mistake = "
              << repeat_mistake << ": "
              << "count: " << metric_pair.first
              << " value: " << metric_pair.second
              << std::endl;
    PCLPointCloud2::Ptr alpha_shape_cloud_upsample(new PCLPointCloud2);
    Io_pcl::loadCloudPLY("../data/alpha_shape_points_upsample_part.ply", *alpha_shape_cloud_upsample);
    metric_pair = building_reconstruction.calculateHoleMetric(ideal_cloud_upsample,
                                                              alpha_shape_cloud_upsample,
                                                              hole_mistake,
                                                              "../data/alpha_shape_hole_part.ply");
    std::cout << std::endl
              << "ideal_points_upsample and alpha_shape hole_metric with max_mistake = " << hole_mistake << ": "
              << "count: " << metric_pair.first
              << " value: " << metric_pair.second
              << std::endl;


    // delaunay
    Io_pcl::loadCloudPLY("../data/delaunay_points_part.ply", *delaunay_cloud);
    metric_pair = building_reconstruction.calculateRepeatabilityMetric(ideal_cloud_upsample,
                                                                       delaunay_cloud,
                                                                       repeat_mistake,
                                                                       "../data/delaunay_not_repeat_part.ply");
    std::cout << std::endl
              << "ideal_points_upsample and delaunay_points repeatability_metric with max_mistake = " << repeat_mistake
              << ": "
              << "count: " << metric_pair.first
              << " value: " << metric_pair.second
              << std::endl;
    PCLPointCloud2::Ptr delaunay_cloud_upsample(new PCLPointCloud2);
    Io_pcl::loadCloudPLY("../data/delaunay_points_upsample_part.ply", *delaunay_cloud_upsample);
    metric_pair = building_reconstruction.calculateHoleMetric(ideal_cloud_upsample,
                                                              delaunay_cloud_upsample,
                                                              hole_mistake,
                                                              "../data/delaunay_hole_part.ply");
    std::cout << std::endl
              << "ideal_points_upsample and delaunay hole_metric with max_mistake = " << hole_mistake << ": "
              << "count: " << metric_pair.first
              << " value: " << metric_pair.second
              << std::endl;


    // scale_space
    Io_pcl::loadCloudPLY("../data/scale_space_points_partnew.ply", *scale_space_cloud);
    metric_pair = building_reconstruction.calculateRepeatabilityMetric(ideal_cloud_upsample,
                                                                       scale_space_cloud,
                                                                       repeat_mistake,
                                                                       "../data/scale_space_not_repeat_partnew.ply");
    std::cout << std::endl << "ideal_points_upsample and scale_space_points repeatability_metric with max_mistake = "
              << repeat_mistake << " 0.3: "
              << "count: " << metric_pair.first
              << " value: " << metric_pair.second
              << std::endl;
    PCLPointCloud2::Ptr scale_space_cloud_upsample(new PCLPointCloud2);
    Io_pcl::loadCloudPLY("../data/scale_space_points_upsample_partnew.ply", *scale_space_cloud_upsample);
    metric_pair = building_reconstruction.calculateHoleMetric(ideal_cloud_upsample,
                                                              scale_space_cloud_upsample,
                                                              hole_mistake,
                                                              "../data/scale_space_hole_partnew.ply");
    std::cout << std::endl << "ideal_points_upsample and scale_space hole_metric with max_mistake = "
              << hole_mistake << " 0.25: "
              << "count: " << metric_pair.first
              << " value: " << metric_pair.second
              << std::endl;


    // greedy
    Io_pcl::loadCloudPLY("../data/greedy_points_partnew.ply", *greedy_cloud);
    metric_pair = building_reconstruction.calculateRepeatabilityMetric(ideal_cloud_upsample,
                                                                       greedy_cloud,
                                                                       repeat_mistake,
                                                                       "../data/greedy_not_repeat_partnew.ply");
    std::cout << std::endl << "ideal_points_upsample and greedy_points repeatability_metric with max_mistake = "
              << repeat_mistake << " 0.3: "
              << "count: " << metric_pair.first
              << " value: " << metric_pair.second
              << std::endl;
    PCLPointCloud2::Ptr greedy_cloud_upsample(new PCLPointCloud2);
    Io_pcl::loadCloudPLY("../data/greedy_points_upsample_partnew.ply", *greedy_cloud_upsample);
    metric_pair = building_reconstruction.calculateHoleMetric(ideal_cloud_upsample,
                                                              greedy_cloud_upsample,
                                                              hole_mistake, //0.25
                                                              "../data/greedy_hole_partnew.ply");
    std::cout << std::endl << "ideal_points_upsample and greedy hole_metric with max_mistake = "
              << hole_mistake << " 0.25: "
              << "count: " << metric_pair.first
              << " value: " << metric_pair.second
              << std::endl;
    return 0;
}

