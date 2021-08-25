#include <math.h>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h>

// This code snippet provides an example of mesh generation from a point cloud with normals.
// It has not been compiled.
pcl::PointCloud<pcl::PointNormal>::Ptr example_cloud;
example_cloud = std::make_shared<pcl::PointCloud<pcl::PointNormal> >();
// Populate your cloud here.
pcl::GreedyProjectionTriangulation<pcl::PointNormal>::Ptr gp3;
gp3 = std::make_shared<pcl::GreedyProjectionTriangulation<pcl::PointNormal> >();
gp3->setMu(2.5);
gp3->setSearchRadius(0.03);
gp3->setMaximumNearestNeighbors(250);
gp3->setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
gp3->setMinimumAngle(M_PI / 18); // 10 degrees
gp3->setMaximumAngle(2 * M_PI / 3); // 120 degrees
//gp3->setNormalConsistency(false);
gp3->setInputCloud(example_cloud);
//pcl::KdTreeFLANN<pcl::PointNormal>::Ptr kdtree (new pcl::KdTreeFLANN<pcl::PointNormal>);
//kdtree->setInputCloud(example_cloud);
//gp3->setSearchMethod(kdtree);
pcl::PolygonMesh mesh;
gp3->reconstruct(mesh);
pcl::io::saveOBJFile("my_mesh_path.obj", mesh);
