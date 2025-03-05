

// //A basic node structure
// #include "rclcpp/rclcpp.hpp"
// #include <octomap/octomap.h>
// #include <octomap_msgs/msg/octomap.hpp>
// #include <octomap/OcTree.h>
// #include <octomap_msgs/conversions.h>

// #include <string>
// #include "std_msgs/msg/string.hpp"


// // #include <octomap_msgs/GetOctomap.h>
// // using octomap_msgs::GetOctomap;
// using namespace octomap;
// // #include <iostream>
// // #include <assert.h>
// using namespace std;
// using std::placeholders::_1;
// // #include "std_msgs/msg/string.hpp"
// // #include <sensor_msgs/msg/point_cloud2.hpp>
// // #include <sensor_msgs/msg/point_cloud2.hpp>

// // #include <octomap_msgs/conversions.h>

// //for marker
// #include <geometry_msgs/msg/point.hpp>
// #include <visualization_msgs/msg/marker.hpp>


// // #include "SdfModel.h"
// #include "my_robot_nbv_cmake/SdfModel.h"

// class MyNode : public rclcpp::Node
// {
//     public:
//         MyNode() 
//         : Node("nbvc_ray_generation"), count_(0) //node的名稱
//         {
//             // RCLCPP_INFO(this->get_logger(), "node nbvc_ray_generation have been started");
            
//             //publisher
//             // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
//             // timer_ = this->create_wall_timer(500ms, std::bind(&MyNode::timer_callback, this));
//             // Create a publisher for the marker
//             marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        
//             //subscriber
//             octomap_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
//                 "/octomap_binary", 10, std::bind(&MyNode::octomap_callback, this, std::placeholders::_1));
//             // // publisher_octomap = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_topicLaLa", 10);
//             // subscription_ = this->create_subscription<std_msgs::msg::String>(
//             //     "topic", 10, std::bind(&MyNode::topic_callback, this, _1));
        
//         }

//     private:
//         // rclcpp::TimerBase::SharedPtr timer_;
//         // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//         size_t count_;
//         rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
//         // Define the subscriber variable as a class member
//         rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription_; //寫在public的那些變數subscribe_啥的都要在這裡先定義好
//         // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

//         void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) //const
//         {
//             RCLCPP_INFO(this->get_logger(), "Received octomap data!!");
//             // Deserialize the binary or full octomap
//             // octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
//             // AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
//             // AbstractOccupancyOcTree* octree = NULL; //AbstractOccupancyOcTree這個class不支援cast_ray
            
            
//             // OcTree* octree = NULL; // 建立一個空的octotree
//             // OcTree octree* = dynamic_cast<OcTree*>(tree);
//             octomap::OcTree* octree = NULL; // octomap::OcTree(0.01);
//             AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
//             // octree = static_cast<octomap::OcTree*>(tree);
//             // OcTree* octree = NULL;
//             // octomap_msgs::msg::Octomap received_msg = *msg; 
//             // AbstractOcTree* tree = octomap_msgs::fullMsgToMap(received_msg);
//             // if( octree != NULL ){  // in-case we are receiving a new octomap before we processed the last one
//             //     delete octree;      // so delete the previous stored object
//             // }
//             // octree = dynamic_cast<OcTree*>(tree);
//             RCLCPP_INFO(this->get_logger(), "aaa");
            
//             // if (octree){
//             //     RCLCPP_INFO(this->get_logger(),"Map received (%zu nodes, %f m res), saving to ", octree->size(), octree->getResolution());
//             //     octree->write("src/dataset/data_octomap/octomap_from_orig.ot"); //AbstractOcTree 是一個octomap的通用格式, 可以自動處理color octomap或是non color octomap. 但它會是以指標的方式存（所以要得到他指到的member要用->而非. 這是c++中的語法) 
//             //     //octovis data/sample.ot 想要看.ot檔要在cd ros2_ws2下用這個(注意相對路徑)
//             //     // Example ray-casting
//             //     octomap::point3d origin(0.0, 0.0, 0.0); // Define the starting point of the ray
//             //     octomap::point3d direction(1.0, 0.0, 0.0); // Define the direction of the ray
//             //     octomap::point3d hitPoint;

//             //     // if (octree->castRay(origin, direction, hitPoint)) {
//             //     //     RCLCPP_INFO(this->get_logger(), "First hit at: (%f, %f, %f)", hitPoint.x(), hitPoint.y(), hitPoint.z());
//             //     // } else {
//             //     //     RCLCPP_INFO(this->get_logger(), "No hit detected along the ray.");
//             //     // }
//             // } else{
//             //     RCLCPP_ERROR(this->get_logger(),"Error reading OcTree from stream");
//             // }

//             // octomap::OcTree octree(0.01);
//             // octomap_msgs::msgToMap(*msg, octree);
//             // OcTree* octree = new OcTree(filename);


//             if (tree){
//                 // octree = dynamic_cast<AbstractOccupancyOcTree*>(tree); //AbstractOccupancyOcTree這個class不支援ray
//                 // octree = dynamic_cast<OcTree*>(tree); //不知道為啥用dynamic的就會出錯
//                 octree = static_cast<octomap::OcTree*>(tree); //用這個static才跑得動 但就要確定msg type要一樣
//                 RCLCPP_INFO(this->get_logger(), "success!!yoyo");

//                 if (octree){
//                     RCLCPP_INFO(this->get_logger(),"Map received (%zu nodes, %f m res), saving to ", octree->size(), octree->getResolution());
//                     octree->write("src/dataset/data_octomap/octomap_from_orig.ot"); //AbstractOcTree 是一個octomap的通用格式, 可以自動處理color octomap或是non color octomap. 但它會是以指標的方式存（所以要得到他指到的member要用->而非. 這是c++中的語法) 
//                     //octovis data/sample.ot 想要看.ot檔要在cd ros2_ws2下用這個(注意相對路徑)
//                     // Example ray-casting
//                     octomap::point3d origin(0.0, 0.0, 0.0); // Define the starting point of the ray
//                     octomap::point3d direction(10, 0, 2.5); // Define the direction of the ray
//                     octomap::point3d hitPoint;
                    
//                     bool hit = octree->castRay(origin, direction, hitPoint, true); //最後一項是ignore unknown point, 要設成true!！不然射到未知的點會變成return false
//                     if (hit) {
//                         RCLCPP_INFO(this->get_logger(), "First hit at: (%f, %f, %f)", hitPoint.x(), hitPoint.y(), hitPoint.z());
//                     } else {
//                         RCLCPP_INFO(this->get_logger(), "No hit detected along the ray.");
//                     }
//                     publish_ray_and_direction(origin, hitPoint, direction);








//                 } else{
//                     RCLCPP_ERROR(this->get_logger(),"Error reading OcTree from stream");
//                 }
//             } else {
//                 RCLCPP_ERROR(this->get_logger(), "Failed to convert Octomap message to OcTree");
//                 RCLCPP_INFO(this->get_logger(), "eee");
//                 // if (resp.map.id == "ColorOcTree")
//                     // ROS_WARN("You requested a binary map for a ColorOcTree - this is currently not supported. Please add -f to request a full map");
//             }

            


        
        
//         }
//         void publish_ray_and_direction(const octomap::point3d &origin, const octomap::point3d &hitPoint, const octomap::point3d &direction) //const
//         {
//             // Marker for the ray (line strip)
//             visualization_msgs::msg::Marker ray_marker;
//             ray_marker.header.frame_id = "odom";   // Change if you have another frame map
//             ray_marker.header.stamp = this->get_clock()->now();
//             ray_marker.ns = "ray_visualization";
//             ray_marker.id = 0;
//             ray_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
//             ray_marker.action = visualization_msgs::msg::Marker::ADD;
//             ray_marker.scale.x = 0.03;  // Line width
//             ray_marker.color.a = 1.0;
//             ray_marker.color.r = 1.0;
//             ray_marker.color.g = 1.0;
//             ray_marker.color.b = 0.0;

//             // Ray points: origin and hit point
//             geometry_msgs::msg::Point p_origin, p_hit;
//             p_origin.x = origin.x();
//             p_origin.y = origin.y();
//             p_origin.z = origin.z();
//             p_hit.x = hitPoint.x();
//             p_hit.y = hitPoint.y();
//             p_hit.z = hitPoint.z();
//             ray_marker.points.push_back(p_origin);
//             ray_marker.points.push_back(p_hit);

//             // Marker for the direction (arrow)
//             visualization_msgs::msg::Marker direction_marker;
//             direction_marker.header.frame_id = "odom"; //map
//             direction_marker.header.stamp = this->get_clock()->now();
//             direction_marker.ns = "direction_visualization";
//             direction_marker.id = 1;
//             direction_marker.type = visualization_msgs::msg::Marker::ARROW;
//             direction_marker.action = visualization_msgs::msg::Marker::ADD;
//             direction_marker.scale.x = 0.01;  // Arrow shaft width 0.05
//             direction_marker.scale.y = 0.1;   // Arrow head width
//             direction_marker.scale.z = 0.1;   // Arrow head length
//             direction_marker.color.a = 1.0;
//             direction_marker.color.r = 0.0;
//             direction_marker.color.g = 1.0;
//             direction_marker.color.b = 0.0;

//             // Set start and end points for the arrow (origin and some distance along the direction)
//             geometry_msgs::msg::Point arrow_end;
//             arrow_end.x = origin.x() + direction.x() * 0.5;  // Make the arrow length customizable
//             arrow_end.y = origin.y() + direction.y() * 0.5;
//             arrow_end.z = origin.z() + direction.z() * 0.5;

//             direction_marker.points.push_back(p_origin);  // Arrow starts at origin
//             direction_marker.points.push_back(arrow_end); // Arrow points along the direction

//             // Publish the markers
//             marker_publisher_->publish(ray_marker);
//             marker_publisher_->publish(direction_marker);
//             RCLCPP_INFO(this->get_logger(), "finish publish marker");
//         }
//         // void timer_callback()
//         // {
//         // auto message = std_msgs::msg::String();
//         // message.data = "Hello, world! " + std::to_string(count_++);
//         // // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//         // publisher_->publish(message);
//         // }
        
//         // void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
//         // {
//         // // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
//         // }
	
// };

// int main(int argc, char **argv)
// {
// 	rclcpp::init(argc, argv);
// 	auto node = std::make_shared<MyNode>();
//     rclcpp::spin(node);
// 	rclcpp::shutdown();
// 	return 0;
// }