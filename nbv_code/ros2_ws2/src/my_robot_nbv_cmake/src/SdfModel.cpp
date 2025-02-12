#include "my_robot_nbv_cmake/SdfModel.h"
// Include your header file
using namespace std;
// #include <open3d/Open3D.h>
// #include <open3d/t/geometry/TriangleMesh.h>
// #include <open3d/t/geometry/RaycastingScene.h>
// #include <open3d/visualization/visualizer/Visualizer.h>
// #include <open3d/core/Tensor.h>
// #include <iostream>
// #include <vector>
// #include <random>
// #include <cmath>


// #include "rclcpp/rclcpp.hpp"

// using namespace std;

// class MyNode : public rclcpp::Node
// {
// 	public:
// 		MyNode() : Node("my_cpp_node")
// 		{
// 			RCLCPP_INFO(this->get_logger(), "Hello from my_cpp_node!");
// 		}
	
// };

//for debug mode


// #define NDEBUG //預設都會沒有define這個, 以後刪掉這個要控制就是控制若是沒define這個就怎

#define DEBUG_MODE false

//在sdf.h那裡
int INPUT_MODE2=3; //1. gazebo big tomato 2. gazebo small tomato 3. realsense


/* Parameters that can Modified: 
signed_distance_threshold //=0.005 //now
*/

// #ifndef NDEBUG
//     #define DEBUG_MODE true
// #else
//     #define DEBUG_MODE false
// #endif

SdfModel::SdfModel(std::shared_ptr<open3d::geometry::PointCloud> pcd, float radius, int num_points)
    : pcd_(pcd), radius_(radius), num_points_(num_points) {
        
    //[check if pcd load success and color it into green]
    // pcd_ = open3d::io::CreatePointCloudFromFile(ply_file_path_);
    if (pcd_ && !pcd_->points_.empty()) {  // Check if the point cloud is not empty
        cout << "File loaded successfully" << endl;
        pcd_->PaintUniformColor(Eigen::Vector3d(0.0, 0.5, 0.5));  // Uniform green color
    } else {
        cout << "Failed to load the file or point cloud is empty" << endl;
    }

    //[turn pcd into mesh and check]
    // mesh0_ = open3d::io::CreateMeshFromFile(ply_file_path_);//現在這裡是一個shared pointer
    //從pcd 去生mesh會歪掉, 導致surface detection很偏離, 所以現在還是用model然後去全轉model
    double alpha = 0.05;
    mesh0_ = open3d::geometry::TriangleMesh::CreateFromPointCloudAlphaShape(*pcd_, alpha); //<Debug> pcd_ 是一個shared的指標, *pcd_ 是拿到指標裡面的內容

    //<補充>
    //Ball pivoting（這個方法要你的pcd 有normal才可以)，所以你這不行: https://www.open3d.org/docs/release/tutorial/geometry/surface_reconstruction.html 
    // float radii = [0.005, 0.01, 0.02, 0.04];
    // vector<double> radii={0.005, 0.01, 0.02, 0.04}; //<Debug> radii要是 Double, 因為CreateFromPointCloudBallPivoting(*pcd_, radii)吃double的
    // mesh0_ = open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(*pcd_, radii);//<Debug> *pcd_ 才是object本身, 如果錯誤顯示的是pcd& 什麼的, 代表它其實要的是object 本身, 你給成指標了, *pcd 才可以得到指標只的內容
    if(DEBUG_MODE){
        open3d::visualization::DrawGeometries({mesh0_});//<Debug: c++中, list, array都要用[]這個！!>////////////
    }
    mesh_ = open3d::t::geometry::TriangleMesh::FromLegacy(*mesh0_);

    //<check>
    // //要吃const open3d::geometry::TriangleMesh& (a reference to a TriangleMesh object) object 本身, 不是pointer
    if (mesh0_ && !mesh0_->IsEmpty()) {  // Check if the point cloud is not empty 
        cout << "mesh0_ File loaded successfully" << endl;

    //// pcd_->PaintUniformColor(Eigen::Vector3d(0.0, 0.5, 0.5));  // Uniform green color
    } else {
        cout << "Failed to load the mesh0_ file or point cloud is empty" << endl;
    }
    if (mesh0_ && !mesh_.IsEmpty()) {  // Check if the point cloud is not empty 
        cout << "mesh_ File loaded successfully" << endl;

    //     // pcd_->PaintUniformColor(Eigen::Vector3d(0.0, 0.5, 0.5));  // Uniform green color
    } else {
        cout << "Failed to load the mesh_ file or point cloud is empty" << endl;
    }

    CreateRaycastingScene();
    ComputeMeshCenter();

}


vector<float> SdfModel::ComputeMeshCenter() {
    auto mesh_center_tensor = mesh_.GetCenter();
    center_x_ = mesh_center_tensor[0].Item<float>();
    center_y_ = mesh_center_tensor[1].Item<float>();
    center_z_ = mesh_center_tensor[2].Item<float>();
    center_ = {center_x_, center_y_, center_z_};

    std::cout << "Center: [" << center_x_ << ", " << center_y_ << ", " << center_z_ << "]" << std::endl;
    return center_;
}


void SdfModel::GenerateSamplePoints() {
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

void SdfModel::ComputeSDFandOccupancy() {
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

//這個輸入一個vector形式的陣列, 計算哪些點點在surface內（順便算出總共幾個點在surface內)
void SdfModel::ComputeSDF(std::vector<Eigen::Vector3f> query_points_input) { //Eigen::Vector3f(x, y, z)
    // cout<< query_points_input;
    // cout<< query_points_input.size();
    // Print the size of the vector
    std::cout << "Number of query points: " << query_points_input.size() << std::endl;
    
    
    for (const auto& point_q : query_points_input) {
        if (DEBUG_MODE) {
            std::cout << "Point: (" << point_q.x() << ", " << point_q.y() << ", " << point_q.z() << ")" << std::endl;
        }
        // auto point_q_formed = open3d::core::Tensor({{point_q.x(), point_q.y(), point_q.z()}}, {1, 3}, open3d::core::Float32);
        // auto point_q_formed = [[point_q.x(), point_q.y(), point_q.z()]];
        //c++做tensor要分開搞, 要先建立vector和shape
        std::vector<float> point_values = {point_q.x(), point_q.y(), point_q.z()};
        open3d::core::SizeVector shape = {1, 3};
        auto point_q_formed = open3d::core::Tensor(point_values, shape, open3d::core::Dtype::Float32);
        auto signed_distance = scene_.ComputeSignedDistance(point_q_formed);
        
        
        
        if (DEBUG_MODE) {
            // Add code you want only in debug mode here
            cout<< std::endl<< "sdf value:"<< signed_distance.Item<float>()<< std::endl;
        }
        
        // auto occupancy = scene_.ComputeOccupancy(point_q_formed);
        
        float signed_distance_threshold = 0.0;
        if(INPUT_MODE2==1){
            signed_distance_threshold=0.05;
        }else{
            signed_distance_threshold=0.005;
        }
        if (signed_distance.Item<float>() <=signed_distance_threshold){//<= 0.05) {  // Inside the mesh //0.038要是in/////////////////////////////////////////////// 20250118
            query_points_in.push_back(point_q);////////////////////////
            AddPoint(point_q.cast<double>(), Eigen::Vector3d(1.0, 0.0, 0.0));  // Red
            //means inside the surface
            
            if (DEBUG_MODE) {
                // Add code you want only in debug mode here
                cout<<"in"<< std::endl;
            }
            
            countPoint[0]+=1;
        } else {
            query_points_out.push_back(point_q);////////////////////////
            AddPoint(point_q.cast<double>(), Eigen::Vector3d(0.0, 0.0, 1.0));  // Blue
            if (DEBUG_MODE) {
                // Add code you want only in debug mode here
                cout<<"out"<< std::endl;
            }
            countPoint[1]+=1;
            
        }
    }
    
}
//==================== [Operation Function] =========================//
array<int, 2> SdfModel::ShowInPointCount(){ //會回傳目前現在有幾個點在surface內
    cout<< "there are "<<countPoint[0] <<"within the surface"; 
    cout<< "there are "<<countPoint[1] <<"outside the surface";
    return countPoint;
}

vector<Eigen::Vector3f> SdfModel::ShowInQueryPoints(){
    return query_points_in;
}

vector<Eigen::Vector3f> SdfModel::ShowOutQueryPoints(){
    return query_points_out;
}

vector<float> SdfModel::GetModelCenter(){
    return center_;
}

// void SdfModel::CleanPointCount(){
//     countPoint[0]=0;//每換一個candidate view要弄一次
//     countPoint[1]=0;
// }


void SdfModel::Visualize() {
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

void SdfModel::ResetCandidatePoint(){//每換一個candidate view要弄一次
    countPoint[0]=0;//每換一個candidate view要弄一次
    countPoint[1]=0;

    //clean 用visualization用的query
    points_.clear();
    colors_.clear();
    query_points_in.clear();
    query_points_out.clear();
    query_points_.clear(); //這個變數其實是下面測試sdf自動生成candidate view的時候用的, 之後就沒用到了

}

//Private function
void SdfModel::CreateRaycastingScene() { //這個做一次就好
    scene_.AddTriangles(mesh_);
    cout<<"success create sdf map"<<endl;
}
void SdfModel::AddPoint(const Eigen::Vector3d& point, const Eigen::Vector3d& color) {
    points_.push_back(point);
    colors_.push_back(color);
    // cout<<endl<<"add point: "<<point;//<<endl<<"color: "<<color;
}







// int main(int argc, char **argv)
// {   //ROS2
// 	rclcpp::init(argc, argv);
// 	// auto node = std::make_shared<MyNode>();
// 	// rclcpp::spin(node);
// 	// rclcpp::shutdown();
//         // Initialize the SdfModel with the file path, radius, and number of points





//     string ply_file_path="src/dataset/data_pcd/TomatoPlant_size_modified_only1tomato_onlyRed.ply";
//     // Load or create a point cloud
//     auto pcd = std::make_shared<open3d::geometry::PointCloud>();   
//     pcd = open3d::io::CreatePointCloudFromFile(ply_file_path); 

//     SdfModel processor(pcd, 0.2f, 5000); //要作為sdf 的model/ 做sample point的radius/ sample點數量



//     // SdfModel processor("src/dataset/data_pcd/TomatoPlant_size_modified_only1tomato_onlyRed.ply", 0.2f, 5000);//_onlyRed.ply

//     // // Load and process data
//     // processor.LoadPointCloud(); //只是為了顯示方便用
//     // processor.LoadMesh();
//     // processor.ComputeMeshCenter();
//     // processor.CreateRaycastingScene();
//     // processor.GenerateSamplePoints();
//     // processor.ComputeSDFandOccupancy();

//     // Eigen::Vector3f p={0.300271, -0.900626, 0.472178};



//     std::vector<Eigen::Vector3f> query_points_input;
//     // int radius= 0.2f;
//     int num_points = 100;
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_real_distribution<> dist_phi(0, 2 * M_PI);
//     // std::uniform_real_distribution<> dist_r(radius, radius);
//     // std::uniform_real_distribution<> dist_theta(-1*M_PI, 1*M_PI); //如果是0的話就是z平面上
//     std::uniform_real_distribution<> dist_theta(0, 0);
//     vector<float> center=processor.ComputeMeshCenter();
//     for (int i = 0; i < num_points; ++i) {
//         float phi = dist_phi(gen);
//         // float r = dist_r(gen);
//         float theta=acos(dist_theta(gen));
        
//         float x = center[0] + 0.05 * sin(theta)*cos(phi);
//         float y = center[1] + 0.05 * sin(theta)*sin(phi);
//         float z = center[2] + 0.05 * cos(theta);

//         query_points_input.emplace_back(Eigen::Vector3f(x, y, z));
//     }
    
//     for (int i = 0; i < 50; ++i) {
//         float phi = dist_phi(gen);
//         // float r = dist_r(gen);
//         float theta=acos(dist_theta(gen));
        
//         float x = center[0] + 0.01 * sin(theta)*cos(phi);
//         float y = center[1] + 0.01 * sin(theta)*sin(phi);
//         float z = center[2] + 0.01 * cos(theta);

//         query_points_input.emplace_back(Eigen::Vector3f(x, y, z));
//     }

//     // query_points_input.emplace_back(Eigen::Vector3f(0.300271, -0.150626,  0.472178));
//     processor.ComputeSDF(query_points_input);
//     // query_points_input.emplace_back(Eigen::Vector3f(0.300271, -0.150626,  0.572178));
//     // query_points_input.emplace_back(Eigen::Vector3f(0.300271, -0.10626,  0.472178));
//     processor.ComputeSDF(query_points_input);
//     array<int, 2> countpoint=processor.ShowInPointCount();
//     cout<<"the count kkk: "<<countpoint[0]<<"[1]: "<<countpoint[1];
//     processor.CleanPointCount();
//     // query_points_input.emplace_back(Eigen::Vector3f(0.300271, -0.18,  0.472178));
//     countpoint=processor.ShowInPointCount();
//     // Visualize results
//     processor.Visualize();




// 	return 0;
// }
