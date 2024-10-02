

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

class MyNode : public rclcpp::Node
{
    public:
        MyNode() 
        : Node("nbvc_ray_generation"), count_(0) //node的名稱
        {
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
        bool firstPCD_ready; //要先確定第一個pcd有資料, 不然在還沒資料的時候直接跑進sdfmodel的initial就會回說媒pcd然後出問題, 所以先卻頂有pcd topic資料進來了再做後面的octomap什麼的
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
        // Define the subscriber variable as a class member
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription_; //寫在public的那些變數subscribe_啥的都要在這裡先定義好
        // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        
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

            // Convert ROS2 PointCloud2 to Open3D PointCloud
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                open3d_cloud_->points_.emplace_back(*iter_x, *iter_y, *iter_z);
            }

            RCLCPP_INFO(this->get_logger(), "Received a new point cloud with %lu points", open3d_cloud_->points_.size());

            firstPCD_ready=true;
            // Now you can access the point cloud in other functions
            // processPointCloud();
        }
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
                        octree->write("src/dataset/data_octomap/octomap_from_orig.ot"); //AbstractOcTree 是一個octomap的通用格式, 可以自動處理color octomap或是non color octomap. 但它會是以指標的方式存（所以要得到他指到的member要用->而非. 這是c++中的語法) 
                        //<Note> octovis data/sample.ot 想要看.ot檔要在cd ros2_ws2下用這個(注意相對路徑)
                        // Example ray-casting
                        // octomap::point3d origin(0.0, 0.0, 0.0); // Define the starting point of the ray
                        // octomap::point3d direction(10, 0, 2.5); // Define the direction of the ray
                        // octomap::point3d hitPoint;

                        //[從這裡建立SdfModel]一個model就是要處理的一個"ModelScene"
                        SdfModel ModelScene(open3d_cloud_, 0.2f, 5000); //這個pcd是要從另一個topic讀過來
                        
                        // vector<float> modelCenter = ModelScene.GetModelCenter();
                        // octomap::point3d modelCenterOctomap_pcd = {modelCenter[0], modelCenter[1], modelCenter[2]};
                        // std::vector<octomap::point3d> origins = RandomPoint(modelCenterOctomap_pcd , 5, 2); //中心座標, 點數, 長度
                        std::vector<octomap::point3d> origins = {{0,0,0},{1,-0.5,0}};
                        int id_k=0;
                        int id_t=10;
                        for (const auto& origin : origins) {
                            id_k++;
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
                            std::vector<octomap::point3d> directions = RandomPoint(origin, 100, 5); //其實不用太長, 反正它只是個方向
                            
                            // Vector to store pairs of directions and their corresponding hit points
                            // std::vector<std::pair<octomap::point3d, octomap::point3d>> rays_and_hits;
                            std::vector<octomap::point3d> hitpointV;
                            std::vector<octomap::point3d> directionsV;

                            for (const auto& direction : directions) {

                                octomap::point3d hitPoint;
                                bool hit = octree->castRay(origin, direction, hitPoint, true);
                                RCLCPP_INFO(this->get_logger(), "Direction: (%f, %f, %f)", direction.x(), direction.y(), direction.z());
                                if (hit) {
                                    RCLCPP_INFO(this->get_logger(), "First hit at: (%f, %f, %f)", hitPoint.x(), hitPoint.y(), hitPoint.z());
                                    // rays_and_hits.push_back(std::make_pair(direction, hitPoint));  // Store the direction and hit point
                                    hitpointV.push_back(hitPoint);
                                    directionsV.push_back(direction);
                                    RCLCPP_INFO(this->get_logger(), "Direction2: (%f, %f, %f)", direction.x(), direction.y(), direction.z());
                                    
                                } else {
                                    RCLCPP_INFO(this->get_logger(), "No hit detected along the ray.");
                                    // If no hit is detected, you can choose to push some default hitPoint or skip it
                                    // rays_and_hits.push_back(std::make_pair(direction, octomap::point3d(0, 0, 0)));  // Default hit point if no hit is detected
                                    // hitpointV.push_back(octomap::point3d(0, 0, 0));
                                    directionsV.push_back(direction);
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
                            publish_any_ray_marker(origin, directionsV, id_t, {0,1,0}, 0.01);
                            
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
                            std::vector<Eigen::Vector3f> eigen_hitpoints = convertToEigenVector(hitpointV);

                            ModelScene.ComputeSDF(eigen_hitpoints);//看現在總共有幾個點要看（因為現在是輸入hitpoint現在就是要看這些打到的點有幾個會在surface內)
                            ModelScene.ShowInPointCount();//會回傳現在有幾個點在surface內
                            std::vector<Eigen::Vector3f> query_points_in = ModelScene.ShowInQueryPoints();
                            
                            // Call the conversion function
                            std::vector<octomath::Vector3> converted_points = convertEigenToOctomap(query_points_in);
                            publish_any_ray_marker(origin, converted_points, id_k, {0,0,1}, 0.02);
                        }
                        
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

        std::vector<octomap::point3d> RandomPoint(octomap::point3d center_, int pointNum, double radius_){
            
            std::vector<octomap::point3d> RayEndPoints;

            std::random_device rd;
            std::mt19937 gen(rd());
            // std::uniform_real_distribution<> dist_phi(-M_PI/4, M_PI/4);//0, 2 * M_PI 0, M_PI: 只有前面
            std::uniform_real_distribution<> dist_phi(-M_PI, M_PI);//整圈////////////////////
            std::uniform_real_distribution<> dist_r(radius_, radius_);
            // std::uniform_real_distribution<> dist_theta(-1*M_PI, 1*M_PI); //如果是0的話就是z平面上
            std::uniform_real_distribution<> dist_theta(-0.5, 0.5);
            for (int i = 0; i < pointNum; ++i) {
                RCLCPP_INFO(this->get_logger(), "%d-th ray", i);
                RCLCPP_INFO(this->get_logger(), " ");
                float phi = dist_phi(gen);
                float r = dist_r(gen);
                float theta=acos(dist_theta(gen));
                
                float x = center_.x() + r * sin(theta)*cos(phi);
                float y = center_.y() + r * sin(theta)*sin(phi);
                float z = center_.z() + r * cos(theta);

                // RayEndPoints.emplace_back(Eigen::Vector3f(x, y, z));
                RayEndPoints.push_back(octomap::point3d(x, y, z));
            }

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