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

// class MyNode : public rclcpp::Node
// {
// 	public:
// 		MyNode() : Node("my_cpp_node")
// 		{
// 			RCLCPP_INFO(this->get_logger(), "Hello from my_cpp_node!");
// 		}
	
// };
class SdfModelV2 {
public:
    SdfModelV2(std::shared_ptr<open3d::geometry::PointCloud> pcd, float radius, int num_points)
        : pcd_(pcd), radius_(radius), num_points_(num_points) {}
        // std::shared_ptr<open3d::geometry::PointCloud> pcd_
    void LoadPointCloud() {
        // pcd_ = open3d::io::CreatePointCloudFromFile(ply_file_path_);
        
        if (pcd_ && !pcd_->points_.empty()) {  // Check if the point cloud is not empty
            cout << "File loaded successfully" << endl;
            pcd_->PaintUniformColor(Eigen::Vector3d(0.0, 0.5, 0.5));  // Uniform green color
        } else {
            cout << "Failed to load the file or point cloud is empty" << endl;
        }
        
    }

    void LoadMesh() {
        // mesh0_ = open3d::io::CreateMeshFromFile(ply_file_path_);//現在這裡是一個shared pointer
        
        //從pcd 去生mesh會歪掉, 導致surface detection很偏離, 所以現在還是用model然後去全轉model
        double alpha = 0.05;
        mesh0_ = open3d::geometry::TriangleMesh::CreateFromPointCloudAlphaShape(*pcd_, alpha); //<Debug> pcd_ 是一個shared的指標, *pcd_ 是拿到指標裡面的內容
        
        
        //Ball pivoting（這個方法要你的pcd 有normal才可以)，所以你這不行: https://www.open3d.org/docs/release/tutorial/geometry/surface_reconstruction.html 
        // float radii = [0.005, 0.01, 0.02, 0.04];
        // vector<double> radii={0.005, 0.01, 0.02, 0.04}; //<Debug> radii要是 Double, 因為CreateFromPointCloudBallPivoting(*pcd_, radii)吃double的
        // mesh0_ = open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(*pcd_, radii);//<Debug> *pcd_ 才是object本身, 如果錯誤顯示的是pcd& 什麼的, 代表它其實要的是object 本身, 你給成指標了, *pcd 才可以得到指標只的內容
        open3d::visualization::DrawGeometries({mesh0_});//<Debug: c++中, list, array都要用[]這個！!>
        
        mesh_ = open3d::t::geometry::TriangleMesh::FromLegacy(*mesh0_);
        // //要吃const open3d::geometry::TriangleMesh& (a reference to a TriangleMesh object) object 本身, 不是pointer
        if (mesh0_ && !mesh0_->IsEmpty()) {  // Check if the point cloud is not empty 
            cout << "mesh0_ File loaded successfully" << endl;

        //     // pcd_->PaintUniformColor(Eigen::Vector3d(0.0, 0.5, 0.5));  // Uniform green color
        } else {
            cout << "Failed to load the mesh0_ file or point cloud is empty" << endl;
        }
        if (mesh0_ && !mesh_.IsEmpty()) {  // Check if the point cloud is not empty 
            cout << "mesh_ File loaded successfully" << endl;

        //     // pcd_->PaintUniformColor(Eigen::Vector3d(0.0, 0.5, 0.5));  // Uniform green color
        } else {
            cout << "Failed to load the mesh_ file or point cloud is empty" << endl;
        }
    }

    void ComputeMeshCenter() {
        auto mesh_center_tensor = mesh_.GetCenter();
        center_x_ = mesh_center_tensor[0].Item<float>();
        center_y_ = mesh_center_tensor[1].Item<float>();
        center_z_ = mesh_center_tensor[2].Item<float>();
        center_ = {center_x_, center_y_, center_z_};

        std::cout << "Center: [" << center_x_ << ", " << center_y_ << ", " << center_z_ << "]" << std::endl;
    }

    void CreateRaycastingScene() {
        scene_.AddTriangles(mesh_);
    }

    void GenerateSamplePoints() {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dist_phi(0, 2 * M_PI);
        std::uniform_real_distribution<> dist_r(0, radius_);
        // std::uniform_real_distribution<> dist_theta(-1*M_PI, 1*M_PI); //如果是0的話就是z平面上
        std::uniform_real_distribution<> dist_theta(0, 0);
        for (int i = 0; i < num_points_; ++i) {
            float phi = dist_phi(gen);
            float r = dist_r(gen);
            float theta=acos(dist_theta(gen));
            
            float x = center_[0] + r * sin(theta)*cos(phi);
            float y = center_[1] + r * sin(theta)*sin(phi);
            float z = center_[2] + r * cos(theta);

            query_points_.emplace_back(Eigen::Vector3f(x, y, z));
        }
    }

    void ComputeSDFandOccupancy() {
        for (const auto& point_q : query_points_) {
            // auto point_q_formed = open3d::core::Tensor({{point_q.x(), point_q.y(), point_q.z()}}, {1, 3}, open3d::core::Float32);
            // auto point_q_formed = [[point_q.x(), point_q.y(), point_q.z()]];
            //c++做tensor要分開搞, 要先建立vector和shap
            std::vector<float> point_values = {point_q.x(), point_q.y(), point_q.z()};
            open3d::core::SizeVector shape = {1, 3};
            auto point_q_formed = open3d::core::Tensor(point_values, shape, open3d::core::Dtype::Float32);
            auto signed_distance = scene_.ComputeSignedDistance(point_q_formed);
            // auto occupancy = scene_.ComputeOccupancy(point_q_formed);
            // if (signed_distance <= 0) {  // Inside the mesh
            //     // AddPoint(point_q.cast<double>(), Eigen::Vector3d(1.0, 0.0, 0.0));  // Red
            //     cout<<"ha";
            // } else {
            //     // AddPoint(point_q.cast<double>(), Eigen::Vector3d(0.0, 0.0, 1.0));  // Blue
            //     cout<<"la";
            // }
            if (signed_distance.Item<float>() <= 0) {  // Inside the mesh
                AddPoint(point_q.cast<double>(), Eigen::Vector3d(1.0, 0.0, 0.0));  // Red
            } else {
                AddPoint(point_q.cast<double>(), Eigen::Vector3d(0.0, 0.0, 1.0));  // Blue
            }
        }
    }

    void Visualize() {
        auto query_point_vis = std::make_shared<open3d::geometry::PointCloud>();
        // query_point_vis->points_ = open3d::utility::Vector3dVector(points_);
        // query_point_vis->colors_ = open3d::utility::Vector3dVector(colors_);
        query_point_vis->points_ = points_;
        query_point_vis->colors_ = colors_;
        // open3d::core::Tensor points_tensor(points_, {static_cast<int64_t>(points_.size()), 3}, open3d::core::Dtype::Float64);
        // open3d::core::Tensor colors_tensor(colors_, {static_cast<int64_t>(colors_.size()), 3}, open3d::core::Dtype::Float64);
    
        // Assign tensors to the PointCloud
        // query_point_vis->points_ = points_tensor; // Using core::Tensor directly
        // query_point_vis->colors_ = colors_tensor; // Using core::Tensor directly

        auto coordinate_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.1, Eigen::Vector3d(center_x_, center_y_, center_z_));

        open3d::visualization::Visualizer vis;
        vis.CreateVisualizerWindow("SDF Value and Sample Point", 640, 480);
        vis.AddGeometry(query_point_vis);
        vis.AddGeometry(pcd_);
        vis.Run();
        vis.DestroyVisualizerWindow();
    }

private:
    // std::string ply_file_path_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_;
    float radius_;
    int num_points_;
    float center_x_, center_y_, center_z_;
    // std::array<float, 3> center_;
    std::vector<float> center_;
    std::vector<Eigen::Vector3d> points_;
    std::vector<Eigen::Vector3d> colors_;
    std::vector<Eigen::Vector3f> query_points_;

    std::shared_ptr<open3d::geometry::TriangleMesh> mesh0_;
    // std::shared_ptr<open3d::t::geometry::TriangleMesh> mesh_;
    
    // std::shared_ptr<open3d::geometry::TriangleMesh> mesh_;
    // open3d::t::geometry::TriangleMesh mesh0_;
    open3d::t::geometry::TriangleMesh mesh_;
    open3d::t::geometry::RaycastingScene scene_;

    void AddPoint(const Eigen::Vector3d& point, const Eigen::Vector3d& color) {
        points_.push_back(point);
        colors_.push_back(color);
    }
};






int main(int argc, char **argv)
{   //ROS2
	rclcpp::init(argc, argv);
	// auto node = std::make_shared<MyNode>();
	// rclcpp::spin(node);
	// rclcpp::shutdown();
        // Initialize the SdfModelV2 with the file path, radius, and number of points





    string ply_file_path="src/dataset/data_pcd/TomatoPlant_size_modified_only1tomato_onlyRed.ply";
    // Load or create a point cloud
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();   
    pcd = open3d::io::CreatePointCloudFromFile(ply_file_path); 

    SdfModelV2 processor(pcd, 0.2f, 5000); //要作為sdf 的model/ 做sample point的radius/ sample點數量



    // SdfModelV2 processor("src/dataset/data_pcd/TomatoPlant_size_modified_only1tomato_onlyRed.ply", 0.2f, 5000);//_onlyRed.ply

    // // Load and process data
    processor.LoadPointCloud(); //只是為了顯示方便用
    processor.LoadMesh();
    processor.ComputeMeshCenter();
    processor.CreateRaycastingScene();
    processor.GenerateSamplePoints();
    processor.ComputeSDFandOccupancy();

    // Visualize results
    processor.Visualize();




	return 0;
}