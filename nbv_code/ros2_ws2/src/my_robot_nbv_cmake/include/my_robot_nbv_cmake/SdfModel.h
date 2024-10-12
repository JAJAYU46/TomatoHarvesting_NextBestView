#ifndef SDF_MODEL_H  // Include guard to prevent multiple inclusions
#define SDF_MODEL_H

#include <open3d/Open3D.h>
#include <open3d/t/geometry/TriangleMesh.h>
#include <open3d/t/geometry/RaycastingScene.h>
#include <open3d/visualization/visualizer/Visualizer.h>
#include <open3d/core/Tensor.h>
#include <iostream>
#include <vector>
#include <random>
#include <cmath>


#include "rclcpp/rclcpp.hpp"

using namespace std;

class SdfModel {
public:
    SdfModel(std::shared_ptr<open3d::geometry::PointCloud> pcd, float radius, int num_points);
        
    vector<float> ComputeMeshCenter();

    void GenerateSamplePoints();

    void ComputeSDFandOccupancy();
    
    void ComputeSDF(std::vector<Eigen::Vector3f> query_points_input);
    //==================== [Operation Function] =========================//
    array<int, 2> ShowInPointCount();
    vector<Eigen::Vector3f> ShowInQueryPoints();
    vector<Eigen::Vector3f> ShowOutQueryPoints();
    vector<float> GetModelCenter();
    void CleanPointCount();
    void Visualize();

private:
    // std::string ply_file_path_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_;
    float radius_;
    int num_points_;
    float center_x_, center_y_, center_z_;

    std::vector<float> center_;
    std::vector<Eigen::Vector3d> points_;
    std::vector<Eigen::Vector3d> colors_;
    std::vector<Eigen::Vector3f> query_points_;
    std::vector<Eigen::Vector3f> query_points_in;
    std::vector<Eigen::Vector3f> query_points_out;

    std::shared_ptr<open3d::geometry::TriangleMesh> mesh0_;
    open3d::t::geometry::TriangleMesh mesh_;
    open3d::t::geometry::RaycastingScene scene_;

    array<int, 2> countPoint={0,0};

    void CreateRaycastingScene();
    void AddPoint(const Eigen::Vector3d& point, const Eigen::Vector3d& color);


};

#endif // SDF_MODEL_H