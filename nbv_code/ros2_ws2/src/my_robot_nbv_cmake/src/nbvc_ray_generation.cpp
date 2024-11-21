

//A basic node structure
#include "rclcpp/rclcpp.hpp"
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <string>
#include "std_msgs/msg/string.hpp"


using namespace octomap;
// #include <iostream>
// #include <assert.h>
using namespace std;
using std::placeholders::_1;


//for marker
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>


// #include "SdfModel.h"
#include "my_robot_nbv_cmake/SdfModel.h"

// //for subscribe pcd from topic
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>



//for pointcloud2 to open3d pcd
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2_eigen/tf2_eigen.h> // For transforming Eigen points

// #include <sensor_msgs/PointCloud2.h>

//for open3d here
// #include "my_robot_nbv_cmake/open3d_conversions.h"

class MyNode : public rclcpp::Node
{
    public:
        MyNode() 
        : Node("nbvc_ray_generation"), count_(0), tf_buffer_(this->get_clock())//, tf_listener_(tf_buffer_) //node的名稱
        {
            
            
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
            // RCLCPP_INFO(this->get_logger(), "node nbvc_ray_generation have been started");
            //<Debug3>記得initialize 指標型class member!！
            // Initialize the Open3D point cloud in the constructor
            open3d_cloud_ = std::make_shared<open3d::geometry::PointCloud>();
            firstPCD_ready = false;
            //publisher
            // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
            // timer_ = this->create_wall_timer(500ms, std::bind(&MyNode::timer_callback, this));
            // Create a publisher for the marker
            marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
            pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visualization_pcd", 10);
        
            //subscriber
            octomap_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
                "/octomap_binary", 10, std::bind(&MyNode::octomap_callback, this, std::placeholders::_1));
            pcd_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/nbv/tompcd_ICPonly", 10, std::bind(&MyNode::pointcloud_callback, this, std::placeholders::_1));
    
            
            // octomap_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            //     "/octomap_binary", 10, std::bind(&MyNode::octomap_callback, this, std::placeholders::_1));
            // // publisher_octomap = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_topicLaLa", 10);
            // subscription_ = this->create_subscription<std_msgs::msg::String>(
            //     "topic", 10, std::bind(&MyNode::topic_callback, this, _1));
        
        }

    private:
        // rclcpp::TimerBase::SharedPtr timer_;
        // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
        tf2_ros::Buffer tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        // TF2 buffer and listener to handle transformations
        // tf2_ros::Buffer tf_buffer_;
        // tf2_ros::TransformListener tf_listener_;
        bool firstPCD_ready; //要先確定第一個pcd有資料, 不然在還沒資料的時候直接跑進sdfmodel的initial就會回說媒pcd然後出問題, 所以先卻頂有pcd topic資料進來了再做後面的octomap什麼的
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
        // Define the subscriber variable as a class member
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription_; //寫在public的那些變數subscribe_啥的都要在這裡先定義好
        // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_subscription_;
        std::shared_ptr<open3d::geometry::PointCloud> open3d_cloud_; //<Debug3> 錯的原因是你在private定義指標型class member都要記得在initialize那邊initialize一次！!
        // auto open3d_cloud_ = std::make_shared<open3d::geometry::PointCloud>(); //<Debug2 c++裡面auto只能用來定義local的變數, 不能用來定義class member
        //<Debug> （錯的, 看debug3)不可以對open3d pointcloud用std::shared_ptr這個, 這個是把一個東西變成指標的形式讓大家去指, (open3d pcd不知道為啥只能當一個物件去做)
        //<Debug> （錯的, 看debug3)不知道為啥open3d pointcloud如果用這個的話build不會出問題, 但是run的時候會直接指出現run fail 還是 run p...啥的, 查了說是因為access到了不該access的地方
        // open3d::geometry::PointCloud open3d_cloud_;//<Debug2> 但你SdfModel是吃auto pcd = std::make_shared<open3d::geometry::PointCloud>(); , 所以也不能直接傳物件
            // Callback to handle incoming PointCloud2 messages
        void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            // // Clear the previous data
            open3d_cloud_->Clear();//<Debug> 物件用. access 一個pointer 裡面的物件才用->

            // // Convert ROS2 PointCloud2 to Open3D PointCloud
            // sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            // sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            // sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

            // for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            //     open3d_cloud_->points_.emplace_back(*iter_x, *iter_y, *iter_z);
            // }

            RCLCPP_INFO(this->get_logger(), "Received a new point cloud with %lu points", open3d_cloud_->points_.size());

            firstPCD_ready=true;
            // Now you can access the point cloud in other functions
            // processPointCloud();
            // sensor_msgs::PointCloud2 ros_pc2;
            // open3d_conversions::open3dToRos(o3d_pc, ros_pc2, msg.header.frame_id);
            // EXPECT_EQ(ros_pc2.height * ros_pc2.width, o3d_pc.points_.size());
            // sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
            // sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
            // sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
            // sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
            // sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
            // sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
            // for (int i = 0; i < 5; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
            // {
            //     const Eigen::Vector3d& point = o3d_pc.points_[i];
            //     EXPECT_EQ(*ros_pc2_x, 0.5 * i);
            //     EXPECT_EQ(*ros_pc2_y, i * i);
            //     EXPECT_EQ(*ros_pc2_z, 10.5 * i);
            //     const Eigen::Vector3d& color = o3d_pc.points_[i];
            //     EXPECT_EQ(*ros_pc2_r, 2 * i);
            //     EXPECT_EQ(*ros_pc2_g, 5 * i);
            //     EXPECT_EQ(*ros_pc2_b, 10 * i);
            // }            
            
            // open3d_cloud_ = ConvertPointCloud2ToOpen3D(msg);
            open3d_cloud_ = pointCloud2ToOpen3D(msg, "odom", tf_buffer_);
            RCLCPP_INFO(this->get_logger(), "Received a new point cloud with %lu points", open3d_cloud_->points_.size());

            // string ply_file_path="src/dataset/data_pcd/TomatoPlant_size_modified_only1tomato_onlyRed.ply";
            // // Load or create a point cloud
            // auto pcd = std::make_shared<open3d::geometry::PointCloud>();   
            // pcd = open3d::io::CreatePointCloudFromFile(ply_file_path); 
            // sensor_msgs::msg::PointCloud2 ros_cloud = open3dToRos2PointCloud2(*pcd, "odom");
            // std::string target_frame = "odom"; // Example of target frame
            // auto open3d_cloud = ConvertPointCloud2ToOpen3D(msg, target_frame, tf_buffer);
            sensor_msgs::msg::PointCloud2 ros_cloud = open3dToRos2PointCloud2(*open3d_cloud_, "odom");//msg->header.frame_id
            pcd_publisher_->publish(ros_cloud);
        }

        //=================================================
        // Function to convert PointCloud2 to Open3D point cloud
        std::shared_ptr<open3d::geometry::PointCloud> pointCloud2ToOpen3D(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr& ros_pc2_msg,
            const std::string& target_frame, 
            tf2_ros::Buffer& tf_buffer) {

            // Create an Open3D point cloud
            auto o3d_pc = std::make_shared<open3d::geometry::PointCloud>();

            // Get the transformation from the PointCloud2 frame to the target frame
            geometry_msgs::msg::TransformStamped transform_stamped;
            try {
                transform_stamped = tf_buffer.lookupTransform(
                    target_frame, ros_pc2_msg->header.frame_id, ros_pc2_msg->header.stamp);
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(rclcpp::get_logger("tf2"), "Transform error: %s", ex.what());
                return nullptr;
            }

            Eigen::Affine3d transform = tf2::transformToEigen(transform_stamped);

            // Iterate through the PointCloud2 data
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*ros_pc2_msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*ros_pc2_msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*ros_pc2_msg, "z");

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                Eigen::Vector3d point(*iter_x, *iter_y, *iter_z);

                // Apply the transformation to each point
                point = transform * point;

                // Add the transformed point to the Open3D point cloud
                o3d_pc->points_.emplace_back(point.x(), point.y(), point.z());
            }

            return o3d_pc;
        }

        // #include <open3d/Open3D.h>
        // #include <sensor_msgs/msg/point_cloud2.hpp>
        // #include <sensor_msgs/point_cloud2_iterator.hpp>
        // #include <rclcpp/rclcpp.hpp>
        // std::shared_ptr<open3d::geometry::PointCloud> ConvertPointCloud2ToOpen3D(const sensor_msgs::msg::PointCloud2::SharedPtr& ros_pointcloud) {
        //     // Create a shared pointer for the Open3D point cloud
        //     auto open3d_cloud = std::make_shared<open3d::geometry::PointCloud>();

        //     // Create iterators to access the PointCloud2 fields
        //     sensor_msgs::PointCloud2ConstIterator<float> iter_x(*ros_pointcloud, "x");
        //     sensor_msgs::PointCloud2ConstIterator<float> iter_y(*ros_pointcloud, "y");
        //     sensor_msgs::PointCloud2ConstIterator<float> iter_z(*ros_pointcloud, "z");

        //     // Check if the PointCloud2 has RGB fields
        //     bool has_rgb = false;
        //     sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(*ros_pointcloud, "r");
        //     sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(*ros_pointcloud, "g");
        //     sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(*ros_pointcloud, "b");
        //     if (ros_pointcloud->fields.size() >= 6) { // RGB fields are present
        //         has_rgb = true;
        //     }

        //     // Loop through the point cloud and extract points (and optionally colors)
        //     for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        //         // Add the points to the Open3D cloud
        //         open3d_cloud->points_.emplace_back(*iter_x, *iter_y, *iter_z);

        //         if (has_rgb) {
        //             // If the point cloud has color, add normalized RGB values
        //             open3d_cloud->colors_.emplace_back(static_cast<double>(*iter_r) / 255.0,
        //                                             static_cast<double>(*iter_g) / 255.0,
        //                                             static_cast<double>(*iter_b) / 255.0);
        //             ++iter_r; ++iter_g; ++iter_b;
        //         }
        //     }

        //     return open3d_cloud;
        // }
        // std::shared_ptr<open3d::geometry::PointCloud> ConvertPointCloud2ToOpen3D(
        //     const sensor_msgs::msg::PointCloud2::SharedPtr& ros_pointcloud,
        //     const std::string& target_frame,
        //     const tf2_ros::Buffer& tf_buffer) {

        //     // Create a shared pointer for the Open3D point cloud
        //     auto open3d_cloud = std::make_shared<open3d::geometry::PointCloud>();

        //     // Extract the frame_id from the PointCloud2 header
        //     std::string source_frame = ros_pointcloud->header.frame_id;

        //     // Transform the point cloud to the target frame if frames differ
        //     if (source_frame != target_frame) {
        //         try {
        //             geometry_msgs::msg::TransformStamped transform_stamped =
        //                 tf_buffer.lookupTransform(target_frame, source_frame, ros_pointcloud->header.stamp);

        //             // Get the transformation as an Eigen matrix
        //             Eigen::Matrix4d transform_matrix = tf2::transformToEigen(transform_stamped).matrix();

        //             // Apply the transformation to all points
        //             for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*ros_pointcloud, "x"),
        //                 iter_y(*ros_pointcloud, "y"), iter_z(*ros_pointcloud, "z");
        //                 iter_x != iter_x.end();
        //                 ++iter_x, ++iter_y, ++iter_z) {

        //                 Eigen::Vector4d point(*iter_x, *iter_y, *iter_z, 1.0);
        //                 Eigen::Vector4d transformed_point = transform_matrix * point;

        //                 // Add the transformed point to Open3D
        //                 open3d_cloud->points_.emplace_back(transformed_point.x(),
        //                                                 transformed_point.y(),
        //                                                 transformed_point.z());
        //             }
        //         } catch (tf2::TransformException& ex) {
        //             RCLCPP_ERROR(rclcpp::get_logger("ConvertPointCloud2ToOpen3D"), "Could not transform: %s", ex.what());
        //             return nullptr; // Handle error appropriately
        //         }
        //     } else {
        //         // No transformation needed, just convert as before
        //         for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*ros_pointcloud, "x"),
        //             iter_y(*ros_pointcloud, "y"), iter_z(*ros_pointcloud, "z");
        //             iter_x != iter_x.end();
        //             ++iter_x, ++iter_y, ++iter_z) {

        //             open3d_cloud->points_.emplace_back(*iter_x, *iter_y, *iter_z);
        //         }
        //     }

        //     return open3d_cloud;
        // }


        sensor_msgs::msg::PointCloud2 open3dToRos2PointCloud2(const open3d::geometry::PointCloud& cloud,
                                                            const std::string& frame_id = "map") {
            sensor_msgs::msg::PointCloud2 ros_cloud;

            // Set the header of the PointCloud2 message
            ros_cloud.header.frame_id = frame_id;
            ros_cloud.header.stamp = rclcpp::Clock().now();

            // Specify the dimensions of the PointCloud2 message
            ros_cloud.height = 1;  // unorganized point cloud
            ros_cloud.width = cloud.points_.size();  // total number of points
            ros_cloud.is_dense = true;
            ros_cloud.is_bigendian = false;

            // Define the fields for the PointCloud2 message
            sensor_msgs::PointCloud2Modifier modifier(ros_cloud);
            modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

            // Resize the data array to fit the point cloud size
            modifier.resize(cloud.points_.size());

            // Iterate through the points and fill the data
            sensor_msgs::PointCloud2Iterator<float> iter_x(ros_cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(ros_cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(ros_cloud, "z");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(ros_cloud, "r");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(ros_cloud, "g");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(ros_cloud, "b");

            for (size_t i = 0; i < cloud.points_.size(); ++i) {
                // Convert XYZ coordinates
                const Eigen::Vector3d& pt = cloud.points_[i];
                *iter_x = pt.x();
                *iter_y = pt.y();
                *iter_z = pt.z();

                // Convert RGB colors if available
                if (cloud.HasColors()) {
                    const Eigen::Vector3d& color = cloud.colors_[i];
                    *iter_r = static_cast<uint8_t>(color.x() * 255);
                    *iter_g = static_cast<uint8_t>(color.y() * 255);
                    *iter_b = static_cast<uint8_t>(color.z() * 255);
                } else {
                    // Set default white color if no color is provided
                    *iter_r = 255;
                    *iter_g = 255;
                    *iter_b = 255;
                }

                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++iter_r;
                ++iter_g;
                ++iter_b;
            }

            return ros_cloud;
        }



        //=================================================
        void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) //const
        {   if(firstPCD_ready==true){

                
                RCLCPP_INFO(this->get_logger(), "Received octomap data!!");
                octomap::OcTree* octree = NULL; // octomap::OcTree(0.01);
                AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
                
                RCLCPP_INFO(this->get_logger(), "aaa");
                
                if (tree){
                    // octree = dynamic_cast<AbstractOccupancyOcTree*>(tree); //AbstractOccupancyOcTree這個class不支援ray
                    // octree = dynamic_cast<OcTree*>(tree); //不知道為啥用dynamic的就會出錯
                    octree = static_cast<octomap::OcTree*>(tree); //用這個static才跑得動 但就要確定msg type要一樣
                    RCLCPP_INFO(this->get_logger(), "success!!yoyo");

                    if (octree){
                        RCLCPP_INFO(this->get_logger(),"Map received (%zu nodes, %f m res), saving to ", octree->size(), octree->getResolution());
                        octree->write("src/dataset/data_octomap/octomap_before_inter.ot"); //AbstractOcTree 是一個octomap的通用格式, 可以自動處理color octomap或是non color octomap. 但它會是以指標的方式存（所以要得到他指到的member要用->而非. 這是c++中的語法) 
                        //<Note> octovis data/sample.ot 想要看.ot檔要在cd ros2_ws2下用這個(注意相對路徑)
                        // Example ray-casting
                        // octomap::point3d origin(0.0, 0.0, 0.0); // Define the starting point of the ray
                        // octomap::point3d direction(10, 0, 2.5); // Define the direction of the ray
                        // octomap::point3d hitPoint;


                        // octree->setOccupancyThres(0.2);  // Set occupancy threshold (default is 0.5)
                        // octree->setClampingThresMin(0.12);  // Minimum clamping threshold
                        // octree->setClampingThresMax(0.97);  // Maximum
                        //把estimated tomato 換成的octomap 和 環境 octomap融合
                        // open3d_cloud_
                          // Iterate through Open3D point cloud and add points to the Octree
                        for (const auto& point : open3d_cloud_->points_) {
                            octree->updateNode(octomap::point3d(point.x(), point.y(), point.z()), true); // Mark as occupied
                        }

                        // Update inner occupancy for the Octree
                        octree->updateInnerOccupancy();
                        octree->write("src/dataset/data_octomap/octomap_after_inter.ot");



                        octomap::point3d BestCandidateView_point;//用來存此輪nbv最佳點
                        int BestCandidateView_gain=0;



                        //[從這裡建立SdfModel]一個model就是要處理的一個"ModelScene"
                        SdfModel ModelScene(open3d_cloud_, 0.2f, 5000); //這個pcd是要從另一個topic讀過來
                        
                        vector<float> modelCenter = ModelScene.GetModelCenter();
                        octomap::point3d modelCenterOctomap_pcd = {modelCenter[0], modelCenter[1], modelCenter[2]};


                        std::vector<octomap::point3d> origins = RandomPoint(modelCenterOctomap_pcd , 20, 0.5); //中心座標, 點數, 長度
                        // std::vector<octomap::point3d> origins = {{0,0,0},{1,-0.5,0.15}};//{0,0,0},{1,-0.5,0.15}};//{1,-0.5,0}
                        int id_k=1;
                        
                        int id_l=200;
                        int id_t=400;
                        for (const auto& origin : origins) {
                            id_k++;
                            id_l++;
                            id_t++;
                            // octomap::point3d origin(0.0, 0.0, 0.0); // Define the starting point of the ray

                            // // Create a vector of directions
                            // std::vector<octomap::point3d> directions = {
                            //     octomap::point3d(10, 0, 2.5),
                            //     octomap::point3d(8, 1, 3),
                            //     octomap::point3d(5, -2, 1),
                            //     octomap::point3d(10, 0, 1.5),
                            //     octomap::point3d(6, 1, 3),
                            //     octomap::point3d(8, 1, 2),
                            //     octomap::point3d(9, 0, 2.5),
                            //     octomap::point3d(7, 1, 3),
                            //     octomap::point3d(6, -2, 1),
                            //     octomap::point3d(5, 0, 1.5),
                            //     octomap::point3d(4, 1, 3),
                            //     octomap::point3d(3, 1, 2)
                            //     };

                            //Generate Random Rays (find random end points)
                            std::vector<octomap::point3d> directions = RandomPoint_Direction(origin, 50, 0.1, modelCenterOctomap_pcd); //modelCenterOctomap_pcd其實不用太長0.5, 反正它只是個方向
                            
                            // Vector to store pairs of directions and their corresponding hit points
                            // std::vector<std::pair<octomap::point3d, octomap::point3d>> rays_and_hits;
                            std::vector<octomap::point3d> hitpointV;
                            std::vector<octomap::point3d> directionsV;

                            for (const auto& direction : directions) {

                                octomap::point3d hitPoint;

                                octomap::point3d realdirection(direction.x() - origin.x(), direction.y() - origin.y(), direction.z() - origin.z()); //<Debug8> 注意, casrRay是要傳進去"realdirection"'向量', 你現在RandomPointDirection算出來的是endpoint不是向量, 因為你publish_any_ray也是傳入一排的endpoint所以綠綠看起才是對的, 但實際計算用的藍藍是錯的
                                // octomap::point3d greendirection(realdirection.x() + origin.x(), realdirection.y() + origin.y(), realdirection.z() + origin.z());
                                // bool hit = octree->castRay(origin, direction, hitPoint, true);
                                bool hit = octree->castRay(origin, realdirection, hitPoint, true);//<Debug8>
                                RCLCPP_INFO(this->get_logger(), "Direction: (%f, %f, %f)", direction.x(), direction.y(), direction.z());
                                if (hit) {
                                    RCLCPP_INFO(this->get_logger(), "First hit at: (%f, %f, %f)", hitPoint.x(), hitPoint.y(), hitPoint.z());
                                    // rays_and_hits.push_back(std::make_pair(direction, hitPoint));  // Store the direction and hit point
                                    hitpointV.push_back(hitPoint);
                                    directionsV.push_back(direction);
                                    // directionsV.push_back(greendirection);
                                    RCLCPP_INFO(this->get_logger(), "Direction2: (%f, %f, %f)", direction.x(), direction.y(), direction.z());
                                    
                                } else {
                                    RCLCPP_INFO(this->get_logger(), "No hit detected along the ray.");
                                    // If no hit is detected, you can choose to push some default hitPoint or skip it
                                    // rays_and_hits.push_back(std::make_pair(direction, octomap::point3d(0, 0, 0)));  // Default hit point if no hit is detected
                                    // hitpointV.push_back(octomap::point3d(0, 0, 0));
                                    // hitpointV.push_back(hitPoint);////////////////之後要刪
                                    directionsV.push_back(direction);
                                    // directionsV.push_back(greendirection);
                                    RCLCPP_INFO(this->get_logger(), "Direction3: (%f, %f, %f)", direction.x(), direction.y(), direction.z());
                                }
                            }
                            // Publish the rays and direction markers
                            // publish_ray_and_direction(origin, rays_and_hits);
                            for (const auto& a :directionsV){
                                RCLCPP_INFO(this->get_logger(), "directionsV: (%f, %f, %f)", a.x(), a.y(), a.z());
                                RCLCPP_INFO(this->get_logger(), "directionsV[1]: (%f, %f, %f)", directionsV[0].x(), directionsV[1].y(), directionsV[2].z());
                            }

                            // publish_multiple_rays_and_directions(origin, hitpointV, directionsV);
                            // publish_any_ray_marker(origin, hitpointV, 0, {1,0,0}); //id要換不然會被蓋掉
                            publish_any_ray_marker(origin, directionsV, id_t, {0,1,0}, 0.003);
                            
                            // bool hit = octree->castRay(origin, direction, hitPoint, true); //<Debug> 最後一項是ignore unknown point, 要設成true!！不然射到未知的點會變成return false
                            // if (hit) {
                            //     RCLCPP_INFO(this->get_logger(), "First hit at: (%f, %f, %f)", hitPoint.x(), hitPoint.y(), hitPoint.z());
                            // } else {
                            //     RCLCPP_INFO(this->get_logger(), "No hit detected along the ray.");
                            // }
                            // publish_ray_and_direction(origin, hitPoint, direction);
                            
                            //把hitpointV輸入到你現在那個SdfModel, 看看這個hit point有沒有在那個surface裡面
                            // Assuming hitpointV is a std::vector<octomath::Vector3>
                            
                            //////////////
                            std::vector<Eigen::Vector3f> eigen_hitpoints = convertToEigenVector(hitpointV);//directions//////////////////////////
                            // std::vector<Eigen::Vector3f> eigen_hitpoints = convertToEigenVector(directions);//directions
                            ModelScene.ComputeSDF(eigen_hitpoints);//看現在總共有幾個點要看（因為現在是輸入hitpoint現在就是要看這些打到的點有幾個會在surface內)
                            array<int, 2> countIn=ModelScene.ShowInPointCount();//會回傳現在有幾個點在surface內

                            int gain=countIn[0]*1;
                            RCLCPP_INFO(this->get_logger(), "There are: (%d) points in the surface", countIn[0]);
                            RCLCPP_INFO(this->get_logger(), "There are: (%d) points out the surface", countIn[1]);
                            
                            RCLCPP_INFO(this->get_logger(), "===============================================");
                            RCLCPP_INFO(this->get_logger(), "The gain for candidate view at (%f, %f, %f) is %d", origin.x(), origin.y(), origin.z(), gain);
                            RCLCPP_INFO(this->get_logger(), "===============================================");

                            //To find best candidate view
                            if(gain>BestCandidateView_gain){
                                BestCandidateView_point=origin;
                                BestCandidateView_gain=gain;
                                publish_point_marker(origin.x(), origin.y(), origin.z(), 0.05, 1.0f, 1.0f, 0.0f);//scale color r g b
    


                            }
                            
                            std::vector<Eigen::Vector3f> query_points_in = ModelScene.ShowInQueryPoints();
                            std::vector<Eigen::Vector3f> query_points_out = ModelScene.ShowOutQueryPoints();
                            
                            // Call the conversion function
                            std::vector<octomath::Vector3> converted_points = convertEigenToOctomap(query_points_in);
                            std::vector<octomath::Vector3> converted_points_out = convertEigenToOctomap(query_points_out);
                            // publish_any_ray_marker(origin, converted_points_out, id_k, {0,0,1}, 0.001);///////////////////////////////////////////////////////////////////////////////////////////////好像又可以了<待處理>不知為啥有三個id組都用就會綠色只能顯示一個的
                            // publish_any_ray_marker(origin, hitpointV, id_k, {0,0,1}, 0.01); //目標, 就是看怎麼讓hitpoint 有備hit 到, 現在是octomap在icp pointcloud會合近來的那邊的grid就是沒辦法hit
                            //hit到的要變成藍色
                            publish_any_ray_marker(origin, converted_points, id_l, {1,0,0}, 0.005);


                            //for debug
                            // octomap::point3d origin(0.0, 0.0, 0.0);
                            // std::vector<octomap::point3d> ray_points;
                            std::vector<octomap::point3d> endPoints_x = {octomap::point3d(10, 0, 0)};
                            std::vector<octomap::point3d> endPoints_y = {octomap::point3d(0, 10, 0)};
                            std::vector<octomap::point3d> endPoints_z = {octomap::point3d(0, 0, 10)};
                            publish_any_ray_marker(octomap::point3d(0,0,0), endPoints_x, 1000, {1,0,0}, 0.005);
                            publish_any_ray_marker(octomap::point3d(0,0,0), endPoints_y, 1001, {1,1,0}, 0.005);
                            publish_any_ray_marker(octomap::point3d(0,0,0), endPoints_z, 1002, {0,0,0.5}, 0.005);
                            // std::vector<octomap::point3d> endPoints_modelCenterOctomap_pcd = {modelCenterOctomap_pcd, {-1,0,0}};
                            // publish_any_ray_marker(octomap::point3d(0,0,0), endPoints_modelCenterOctomap_pcd, 8, {1,0,1}, 0.06);
                            
                        }
                        
                        RCLCPP_INFO(this->get_logger(), "===============================================");
                        RCLCPP_INFO(this->get_logger(), "The Best candidate view for this scene is at (%f, %f, %f) \n with gain=%d", BestCandidateView_point.x(), BestCandidateView_point.y(), BestCandidateView_point.z(), BestCandidateView_gain);
                        RCLCPP_INFO(this->get_logger(), "===============================================");

                        ModelScene.Visualize();



                    } else{
                        RCLCPP_ERROR(this->get_logger(),"Error reading OcTree from stream");
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to convert Octomap message to OcTree");
                    RCLCPP_INFO(this->get_logger(), "eee");
                    // if (resp.map.id == "ColorOcTree")
                        // ROS_WARN("You requested a binary map for a ColorOcTree - this is currently not supported. Please add -f to request a full map");
                }

                
            }

        
        
        }

        void publish_point_marker(float x, float y, float z, float scale=0.05, float r=1, float g=0, float b=0, float a=1) {
            visualization_msgs::msg::Marker marker;

            // Set frame_id and timestamp
            marker.header.frame_id = "odom";  // Make sure this frame exists in your tf tree
            marker.header.stamp = this->get_clock()->now();

            // Set the namespace and id for this marker
            marker.ns = "my_point_marker";
            marker.id = 0;

            // Set the marker type
            marker.type = visualization_msgs::msg::Marker::SPHERE;

            // Set the action: add the marker
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set the position of the marker (huge point)
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;

            // Set the scale of the marker (this will make it a "huge" point)
            marker.scale.x = scale;
            marker.scale.y = scale;
            marker.scale.z = scale;

            // Set the color (RGBA)
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = a;

            // Publish the marker 
            marker_publisher_->publish(marker);
        }
        std::vector<octomap::point3d> RandomPoint(octomap::point3d center_, int pointNum, double radius_){
            
            std::vector<octomap::point3d> RayEndPoints;

            std::random_device rd;
            std::mt19937 gen(rd());
            // std::uniform_real_distribution<> dist_phi(-M_PI/4, M_PI/4);//0, 2 * M_PI 0, M_PI: 只有前面
            std::uniform_real_distribution<> dist_phi(-M_PI, M_PI);//整圈////////////////////
            std::uniform_real_distribution<> dist_r(radius_, radius_);
            // std::uniform_real_distribution<> dist_theta(-1*M_PI, 1*M_PI); //如果是0的話就是z平面上
            // std::uniform_real_distribution<> dist_theta(0, 0.5);//只要上半球
            std::uniform_real_distribution<> dist_cos_theta(0, 1.0); //<chatgpt2>



            for (int i = 0; i < pointNum; ++i) {
                RCLCPP_INFO(this->get_logger(), "%d-th ray", i);
                RCLCPP_INFO(this->get_logger(), " ");
                float phi = dist_phi(gen);
                float r = dist_r(gen);
                // float theta=acos(dist_theta(gen)); //<chatgpt2> 說這樣會不能uniform, 因為acos不是uniform 的, 所以改用上下面chatgpt2
                float theta = acos(dist_cos_theta(gen)); //<chatgpt2>

                
                float x = center_.x() + r * sin(theta)*cos(phi);
                float y = center_.y() + r * sin(theta)*sin(phi);
                float z = center_.z() + r * cos(theta);

                // RayEndPoints.emplace_back(Eigen::Vector3f(x, y, z));
                RayEndPoints.push_back(octomap::point3d(x, y, z));
            }

            return RayEndPoints;

        }

        std::vector<octomap::point3d> RandomPoint_Direction(octomap::point3d center_, int pointNum, double radius_, octomap::point3d torwordP={1,0,0}){ //gazebo座標最前方是yx?？x?
            //RandomPoint_Direction 以center_ torwardP vector 為中心去建立那個ray
            std::vector<octomap::point3d> RayEndPoints;

            //==============================
            //  octomap::point3d torwordP={1,0,0};
            //現在中心是沿著x軸去generate 那個ray, 但是我希望是以center_ torwardP vector 為中心去建立那個ray, 往下什麼的也是, 所以要把這些end point做旋轉（因為往下也是, 所以要用旋轉的)
            Eigen::Vector3f direction(torwordP.x() - center_.x(), torwordP.y() - center_.y(), torwordP.z() - center_.z());
            // direction.normalize(); // Normalize the direction vector

            // Direction vector components
            float dx = torwordP.x() - center_.x();
            float dy = torwordP.y() - center_.y();
            float dz = torwordP.z() - center_.z();

            // Magnitude of the direction vector (r)
            float r = std::sqrt(dx * dx + dy * dy + dz * dz);

            // Calculate phi (azimuthal angle)
            // if(){
            //     float torwardP_phi = std::atan(dy/dx);  // atan2 gives phi in the range [-PI, PI]
            // }
            // float torwardP_phi=atan(dy/dx);
            float torwardP_phi = atan2(dy, dx);  // <Debug9> 加了綠綠的方向就正常了ㄟ好神奇喔<chatgpt> 說這樣可以generate整個circleatan2 handles full circular range
            // if(dx==0){
            //     torwardP_phi = M_PI/2;  // atan2 gives phi in the range [-PI, PI]
            // }else{
            //     torwardP_phi = std::atan(dy/dx);
            // }
            
            

            // Calculate theta (polar angle)
            // float torwardP_theta = std::acos(dz / r); // acos gives theta in the range [0, PI]
            float torwardP_theta = -asin(dz/r);
            //==============================
            //[structured ray] <待處理> 還沒改, 原本的random ray已經留在下方了
            
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dist_phi(-M_PI/4, M_PI/4);//0, 2 * M_PI 0, M_PI: 只有前面
            // std::uniform_real_distribution<> dist_phi(0, 0);
            // std::uniform_real_distribution<> dist_phi(-M_PI, M_PI);//整圈////////////////////
            std::uniform_real_distribution<> dist_r(radius_, radius_);
            // std::uniform_real_distribution<> dist_theta(-1*M_PI, 1*M_PI); //如果是0的話就是z平面上
            std::uniform_real_distribution<> dist_theta(-0.5, 0.5);
            // std::uniform_real_distribution<> dist_theta(0, 0);

            // float raytheta
            // RayEndPoints.push_back(octomap::point3d(center_.x(), center_.y(), center_.z()));//////////////////////////////////
            for (int i = 0; i < pointNum; ++i) {
                RCLCPP_INFO(this->get_logger(), "%d-th ray", i);
                RCLCPP_INFO(this->get_logger(), " ");
                float phi = dist_phi(gen)+torwardP_phi;
                float r = dist_r(gen);
                float theta=acos(dist_theta(gen))+torwardP_theta;
                
                float x = center_.x() + r * sin(theta)*cos(phi);
                float y = center_.y() + r * sin(theta)*sin(phi);
                float z = center_.z() + r * cos(theta);

                // RayEndPoints.emplace_back(Eigen::Vector3f(x, y, z));
                RayEndPoints.push_back(octomap::point3d(x, y, z));
            }

            //=======================================
            //[random ray]
            // std::random_device rd;
            // std::mt19937 gen(rd());
            // std::uniform_real_distribution<> dist_phi(-M_PI/4, M_PI/4);//0, 2 * M_PI 0, M_PI: 只有前面
            // // std::uniform_real_distribution<> dist_phi(0, 0);
            // // std::uniform_real_distribution<> dist_phi(-M_PI, M_PI);//整圈////////////////////
            // std::uniform_real_distribution<> dist_r(radius_, radius_);
            // // std::uniform_real_distribution<> dist_theta(-1*M_PI, 1*M_PI); //如果是0的話就是z平面上
            // std::uniform_real_distribution<> dist_theta(-0.5, 0.5);
            // // std::uniform_real_distribution<> dist_theta(0, 0);
            // // RayEndPoints.push_back(octomap::point3d(center_.x(), center_.y(), center_.z()));//////////////////////////////////
            // for (int i = 0; i < pointNum; ++i) {
            //     RCLCPP_INFO(this->get_logger(), "%d-th ray", i);
            //     RCLCPP_INFO(this->get_logger(), " ");
            //     float phi = dist_phi(gen)+torwardP_phi;
            //     float r = dist_r(gen);
            //     float theta=acos(dist_theta(gen))+torwardP_theta;
                
            //     float x = center_.x() + r * sin(theta)*cos(phi);
            //     float y = center_.y() + r * sin(theta)*sin(phi);
            //     float z = center_.z() + r * cos(theta);

            //     // RayEndPoints.emplace_back(Eigen::Vector3f(x, y, z));
            //     RayEndPoints.push_back(octomap::point3d(x, y, z));
            // }
            //=============================================================
            //這邊製造的是ray end points 不是direction!！要剪掉自己才會變成direction!！!
            return RayEndPoints;

        }

        // Helper function to convert octomath::Vector3 to Eigen::Vector3f
        std::vector<Eigen::Vector3f> convertToEigenVector(const std::vector<octomath::Vector3>& octomap_vec) {
            std::vector<Eigen::Vector3f> eigen_vec;
            eigen_vec.reserve(octomap_vec.size());

            for (const auto& vec : octomap_vec) {
                // Convert each octomath::Vector3 to Eigen::Vector3f
                eigen_vec.emplace_back(vec.x(), vec.y(), vec.z());
            }

            return eigen_vec;
        }
        // Function to convert Eigen::Matrix<float, 3, 1> points to octomath::Vector3
        std::vector<octomath::Vector3> convertEigenToOctomap(const std::vector<Eigen::Matrix<float, 3, 1>>& eigen_points) {
            std::vector<octomath::Vector3> octomap_points;

            // Convert each Eigen::Matrix<float, 3, 1> to octomath::Vector3
            for (const auto& point : eigen_points) {
                octomap_points.emplace_back(point.x(), point.y(), point.z());
            }

            return octomap_points;
        }



        //原本的function, 現在被publish_any_ray取代了
        // void publish_multiple_rays_and_directions(const octomap::point3d &origin, std::vector<octomap::point3d> &hitPoints, std::vector<octomap::point3d> &directions) {
        //     visualization_msgs::msg::Marker ray_marker;
        //     visualization_msgs::msg::Marker direction_marker;

        //     for (const auto& b :directions){
        //         RCLCPP_INFO(this->get_logger(), "directions: (%f, %f, %f)", b.x(), b.y(), b.z());
        //         RCLCPP_INFO(this->get_logger(), "directions[1]: (%f, %f, %f)", directions[0].x(), directions[1].y(), directions[2].z());
        //     }
        //     // for (size_t i = 0; i < rays_and_hits.size(); ++i)
        //     // {
        //         // const octomap::point3d &direction = rays_and_hits[i].first;
        //         // const octomap::point3d &hitPoint = rays_and_hits[i].second;

        //         // Marker for the ray (line strip)
        //         // visualization_msgs::msg::Marker ray_marker;
        //     ray_marker.header.frame_id = "odom";   // Change if you have another frame map
        //     ray_marker.header.stamp = this->get_clock()->now();
        //     ray_marker.ns = "ray_visualization";
        //     ray_marker.id = 0;  // Unique ID for each marker
        //     // ray_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        //     ray_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        //     ray_marker.action = visualization_msgs::msg::Marker::ADD;
        //     ray_marker.scale.x = 0.03;  // Line width
        //     ray_marker.color.a = 1.0;
        //     ray_marker.color.r = 1.0;
        //     ray_marker.color.g = 1.0;
        //     ray_marker.color.b = 0.0;

        //         // Ray points: origin and hit point
        //         // geometry_msgs::msg::Point p_origin, p_hit;
        //         // p_origin.x = origin.x();
        //         // p_origin.y = origin.y();
        //         // p_origin.z = origin.z();
        //         // p_hit.x = hitPoint.x();
        //         // p_hit.y = hitPoint.y();
        //         // p_hit.z = hitPoint.z();
        //         // ray_marker.points.push_back(p_origin);
        //         // ray_marker.points.push_back(p_hit);

        //         // Marker for the direction (arrow)
        //         // visualization_msgs::msg::Marker direction_marker;
        //     direction_marker.header.frame_id = "odom"; //map
        //     direction_marker.header.stamp = this->get_clock()->now();
        //     direction_marker.ns = "direction_visualization";
        //     // direction_marker.id = rays_and_hits.size() + i;  // Unique ID for the arrow marker
        //     direction_marker.id = 1;
        //     direction_marker.type = visualization_msgs::msg::Marker::LINE_LIST;//ARROW;
        //     direction_marker.action = visualization_msgs::msg::Marker::ADD;
        //     direction_marker.scale.x = 0.01;  // Arrow shaft width 0.05  0.01
        //     direction_marker.scale.y = 0.1;   // Arrow head width
        //     direction_marker.scale.z = 0.1;   // Arrow head length
        //     direction_marker.color.a = 1.0;
        //     direction_marker.color.r = 0.0;
        //     direction_marker.color.g = 1.0;
        //     direction_marker.color.b = 0.0;

        //         // // Set start and end points for the arrow (origin and some distance along the direction)
        //         // geometry_msgs::msg::Point arrow_end;
        //         // arrow_end.x = origin.x() + direction.x() * 0.5;  // Make the arrow length customizable
        //         // arrow_end.y = origin.y() + direction.y() * 0.5;
        //         // arrow_end.z = origin.z() + direction.z() * 0.5;

        //         // direction_marker.points.push_back(p_origin);  // Arrow starts at origin
        //         // direction_marker.points.push_back(arrow_end); // Arrow points along the direction

        //     // geometry_msgs::msg::Point p_origin, p_hit, arrow_end;
        //     for (size_t i = 0; i < directions.size(); ++i) {//hitPoints
        //         geometry_msgs::msg::Point p_origin, p_hit, arrow_end;

        //         // Set origin point
        //         p_origin.x = origin.x();
        //         p_origin.y = origin.y();
        //         p_origin.z = origin.z();

        //         // Set hit point (end of ray)
        //         p_hit.x = hitPoints[i].x();
        //         p_hit.y = hitPoints[i].y();
        //         p_hit.z = hitPoints[i].z();

        //         // Add both origin and hit point to ray_marker (for LINE_LIST)
        //         ray_marker.points.push_back(p_origin);
        //         ray_marker.points.push_back(p_hit);

        //         // Set direction arrow end
        //         arrow_end.x = origin.x() + directions[i].x() * 0.5;  // Customize the length of the arrow
        //         arrow_end.y = origin.y() + directions[i].y() * 0.5; //0.5
        //         arrow_end.z = origin.z() + directions[i].z() * 0.5;
        //         RCLCPP_INFO(this->get_logger(), "directions[i] (%f, %f, %f)", directions[i].x(), directions[i].y(), directions[i].z());
        //         // Add arrow points (origin -> direction)
        //         RCLCPP_INFO(this->get_logger(), "arrow_end: (%f, %f, %f)", arrow_end.x, arrow_end.y, arrow_end.z);
        //         direction_marker.points.push_back(p_origin);  // Arrow starts at origin
        //         direction_marker.points.push_back(arrow_end); // Arrow points along the direction
        //     }
        //         // // Publish the markers
        //         // marker_publisher_->publish(ray_marker);
        //         // marker_publisher_->publish(direction_marker);
        //     // }
        //     // Publish the markers
        //     //<Debug> 下面兩行要放在loop外面才publish不然會publish完又重新被弄調啥的, 反正會指出限三個ray
        //     marker_publisher_->publish(ray_marker);
        //     marker_publisher_->publish(direction_marker);
        //     RCLCPP_INFO(this->get_logger(), "Finished publishing all markers.");
            
        // }
        void publish_any_ray_marker(const octomap::point3d &origin, std::vector<octomap::point3d> &endPoints, int id=0,const std::array<double, 3> &marker_color={1,0,0}, const double x_scale=0.03) {
            visualization_msgs::msg::Marker any_ray_marker;
            
            // for (const auto& b :directions){
            //     RCLCPP_INFO(this->get_logger(), "directions: (%f, %f, %f)", b.x(), b.y(), b.z());
            //     RCLCPP_INFO(this->get_logger(), "directions[1]: (%f, %f, %f)", directions[0].x(), directions[1].y(), directions[2].z());
            // }
            // for (size_t i = 0; i < rays_and_hits.size(); ++i)
            // {
                // const octomap::point3d &direction = rays_and_hits[i].first;
                // const octomap::point3d &hitPoint = rays_and_hits[i].second;

                // Marker for the ray (line strip)
                // visualization_msgs::msg::Marker ray_marker;
            any_ray_marker.header.frame_id = "odom";   // Change if you have another frame map
            any_ray_marker.header.stamp = this->get_clock()->now();
            any_ray_marker.ns = "any_ray_visualization";
            any_ray_marker.id = id;  // Unique ID for each marker  #<Note> 不同次的的publish如果ID同就會被替代掉
            // ray_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            any_ray_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            any_ray_marker.action = visualization_msgs::msg::Marker::ADD;
            any_ray_marker.scale.x = x_scale;  // Line width
            any_ray_marker.color.a = 1.0;
            any_ray_marker.color.r = marker_color[0];
            any_ray_marker.color.g = marker_color[1];
            any_ray_marker.color.b = marker_color[2];

                // Ray points: origin and hit point
                // geometry_msgs::msg::Point p_origin, p_hit;
                // p_origin.x = origin.x();
                // p_origin.y = origin.y();
                // p_origin.z = origin.z();
                // p_hit.x = hitPoint.x();
                // p_hit.y = hitPoint.y();
                // p_hit.z = hitPoint.z();
                // ray_marker.points.push_back(p_origin);
                // ray_marker.points.push_back(p_hit);

                // Marker for the direction (arrow)
                // visualization_msgs::msg::Marker direction_marker;

                // // Set start and end points for the arrow (origin and some distance along the direction)
                // geometry_msgs::msg::Point arrow_end;
                // arrow_end.x = origin.x() + direction.x() * 0.5;  // Make the arrow length customizable
                // arrow_end.y = origin.y() + direction.y() * 0.5;
                // arrow_end.z = origin.z() + direction.z() * 0.5;

                // direction_marker.points.push_back(p_origin);  // Arrow starts at origin
                // direction_marker.points.push_back(arrow_end); // Arrow points along the direction

            // geometry_msgs::msg::Point p_origin, p_hit, arrow_end;
            for (size_t i = 0; i < endPoints.size(); ++i) {//hitPoints
                geometry_msgs::msg::Point p_origin, p_hit, arrow_end;

                // Set origin point
                p_origin.x = origin.x();
                p_origin.y = origin.y();
                p_origin.z = origin.z();

                // Set hit point (end of ray)
                p_hit.x = endPoints[i].x();
                p_hit.y = endPoints[i].y();
                p_hit.z = endPoints[i].z();

                // Add both origin and hit point to ray_marker (for LINE_LIST)
                any_ray_marker.points.push_back(p_origin);
                any_ray_marker.points.push_back(p_hit);

            }
                // // Publish the markers
                // marker_publisher_->publish(ray_marker);
                // marker_publisher_->publish(direction_marker);
            // }
            // Publish the markers
            //<Debug> 下面兩行要放在loop外面才publish不然會publish完又重新被弄調啥的, 反正會指出限三個ray
            marker_publisher_->publish(any_ray_marker);
            RCLCPP_INFO(this->get_logger(), "Finished publishing any_ray_marker.");
            
        }

        
	
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}