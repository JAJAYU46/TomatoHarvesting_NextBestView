

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

class MyNode : public rclcpp::Node
{
    public:
        MyNode() 
        : Node("nbvc_ray_generationV3_beforeclean"), count_(0) //node的名稱
        {
            // RCLCPP_INFO(this->get_logger(), "node nbvc_ray_generationV3_beforeclean have been started");
            
            //publisher
            // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
            // timer_ = this->create_wall_timer(500ms, std::bind(&MyNode::timer_callback, this));
            // Create a publisher for the marker
            marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        
            //subscriber
            octomap_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
                "/octomap_binary", 10, std::bind(&MyNode::octomap_callback, this, std::placeholders::_1));
            // // publisher_octomap = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_topicLaLa", 10);
            // subscription_ = this->create_subscription<std_msgs::msg::String>(
            //     "topic", 10, std::bind(&MyNode::topic_callback, this, _1));
        
        }

    private:
        // rclcpp::TimerBase::SharedPtr timer_;
        // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
        // Define the subscriber variable as a class member
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription_; //寫在public的那些變數subscribe_啥的都要在這裡先定義好
        // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

        void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) //const
        {
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
                    //octovis data/sample.ot 想要看.ot檔要在cd ros2_ws2下用這個(注意相對路徑)
                    // Example ray-casting
                    // octomap::point3d origin(0.0, 0.0, 0.0); // Define the starting point of the ray
                    // octomap::point3d direction(10, 0, 2.5); // Define the direction of the ray
                    // octomap::point3d hitPoint;


                    octomap::point3d origin(0.0, 0.0, 0.0); // Define the starting point of the ray

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
                    std::vector<octomap::point3d> directions = RandomPoint(origin, 50, 5); //其實不用太長, 反正它只是個方向
                    // std::vector<octomap::point3d> RandomPoint(octomap::point3d center_, int pointNum, double radius_){
                    //========================================================
                    // std::vector<octomap::point3d> directions;

                    // std::random_device rd;
                    // std::mt19937 gen(rd());
                    // std::uniform_real_distribution<> dist_phi(-M_PI/2, M_PI/2);//0, 2 * M_PI 0, M_PI: 只有前面
                    // std::uniform_real_distribution<> dist_r(10, 10);
                    // // std::uniform_real_distribution<> dist_theta(-1*M_PI, 1*M_PI); //如果是0的話就是z平面上
                    // std::uniform_real_distribution<> dist_theta(0, 1);
                    // for (int i = 0; i < 10; ++i) {
                    //     RCLCPP_INFO(this->get_logger(), "%d-th ray", i);
                    //     RCLCPP_INFO(this->get_logger(), " ");
                    //     float phi = dist_phi(gen);
                    //     float r = dist_r(gen);
                    //     float theta=acos(dist_theta(gen));
                        
                    //     float x = origin.x() + r * sin(theta)*cos(phi);
                    //     float y = origin.y() + r * sin(theta)*sin(phi);
                    //     float z = origin.z() + r * cos(theta);

                    //     // RayEndPoints.emplace_back(Eigen::Vector3f(x, y, z));
                    //     directions.push_back(octomap::point3d(x, y, z));
                    // }




                    //========================================================



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
                            hitpointV.push_back(octomap::point3d(0, 0, 0));
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

                    publish_multiple_rays_and_directions(origin, hitpointV, directionsV);
                    // bool hit = octree->castRay(origin, direction, hitPoint, true); //最後一項是ignore unknown point, 要設成true!！不然射到未知的點會變成return false
                    // if (hit) {
                    //     RCLCPP_INFO(this->get_logger(), "First hit at: (%f, %f, %f)", hitPoint.x(), hitPoint.y(), hitPoint.z());
                    // } else {
                    //     RCLCPP_INFO(this->get_logger(), "No hit detected along the ray.");
                    // }
                    // publish_ray_and_direction(origin, hitPoint, direction);



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

        std::vector<octomap::point3d> RandomPoint(octomap::point3d center_, int pointNum, double radius_){
            
            std::vector<octomap::point3d> RayEndPoints;

            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dist_phi(-M_PI/4, M_PI/4);//0, 2 * M_PI 0, M_PI: 只有前面
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

        // void publish_ray_and_direction(const octomap::point3d &origin, const std::vector<std::pair<octomap::point3d, octomap::point3d>> &rays_and_hits) //const
        // {
        void publish_multiple_rays_and_directions(const octomap::point3d &origin, std::vector<octomap::point3d> &hitPoints, std::vector<octomap::point3d> &directions) {
            visualization_msgs::msg::Marker ray_marker;
            visualization_msgs::msg::Marker direction_marker;

            for (const auto& b :directions){
                RCLCPP_INFO(this->get_logger(), "directions: (%f, %f, %f)", b.x(), b.y(), b.z());
                RCLCPP_INFO(this->get_logger(), "directions[1]: (%f, %f, %f)", directions[0].x(), directions[1].y(), directions[2].z());
            }
            // for (size_t i = 0; i < rays_and_hits.size(); ++i)
            // {
                // const octomap::point3d &direction = rays_and_hits[i].first;
                // const octomap::point3d &hitPoint = rays_and_hits[i].second;

                // Marker for the ray (line strip)
                // visualization_msgs::msg::Marker ray_marker;
            ray_marker.header.frame_id = "odom";   // Change if you have another frame map
            ray_marker.header.stamp = this->get_clock()->now();
            ray_marker.ns = "ray_visualization";
            ray_marker.id = 0;  // Unique ID for each marker
            // ray_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            ray_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            ray_marker.action = visualization_msgs::msg::Marker::ADD;
            ray_marker.scale.x = 0.03;  // Line width
            ray_marker.color.a = 1.0;
            ray_marker.color.r = 1.0;
            ray_marker.color.g = 1.0;
            ray_marker.color.b = 0.0;

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
            direction_marker.header.frame_id = "odom"; //map
            direction_marker.header.stamp = this->get_clock()->now();
            direction_marker.ns = "direction_visualization";
            // direction_marker.id = rays_and_hits.size() + i;  // Unique ID for the arrow marker
            direction_marker.id = 1;
            direction_marker.type = visualization_msgs::msg::Marker::LINE_LIST;//ARROW;
            direction_marker.action = visualization_msgs::msg::Marker::ADD;
            direction_marker.scale.x = 0.01;  // Arrow shaft width 0.05  0.01
            direction_marker.scale.y = 0.1;   // Arrow head width
            direction_marker.scale.z = 0.1;   // Arrow head length
            direction_marker.color.a = 1.0;
            direction_marker.color.r = 0.0;
            direction_marker.color.g = 1.0;
            direction_marker.color.b = 0.0;

                // // Set start and end points for the arrow (origin and some distance along the direction)
                // geometry_msgs::msg::Point arrow_end;
                // arrow_end.x = origin.x() + direction.x() * 0.5;  // Make the arrow length customizable
                // arrow_end.y = origin.y() + direction.y() * 0.5;
                // arrow_end.z = origin.z() + direction.z() * 0.5;

                // direction_marker.points.push_back(p_origin);  // Arrow starts at origin
                // direction_marker.points.push_back(arrow_end); // Arrow points along the direction

            // geometry_msgs::msg::Point p_origin, p_hit, arrow_end;
            for (size_t i = 0; i < directions.size(); ++i) {//hitPoints
                geometry_msgs::msg::Point p_origin, p_hit, arrow_end;

                // Set origin point
                p_origin.x = origin.x();
                p_origin.y = origin.y();
                p_origin.z = origin.z();

                // Set hit point (end of ray)
                p_hit.x = hitPoints[i].x();
                p_hit.y = hitPoints[i].y();
                p_hit.z = hitPoints[i].z();

                // Add both origin and hit point to ray_marker (for LINE_LIST)
                ray_marker.points.push_back(p_origin);
                ray_marker.points.push_back(p_hit);

                // Set direction arrow end
                arrow_end.x = origin.x() + directions[i].x() * 0.5;  // Customize the length of the arrow
                arrow_end.y = origin.y() + directions[i].y() * 0.5; //0.5
                arrow_end.z = origin.z() + directions[i].z() * 0.5;
                RCLCPP_INFO(this->get_logger(), "directions[i] (%f, %f, %f)", directions[i].x(), directions[i].y(), directions[i].z());
                // Add arrow points (origin -> direction)
                RCLCPP_INFO(this->get_logger(), "arrow_end: (%f, %f, %f)", arrow_end.x, arrow_end.y, arrow_end.z);
                direction_marker.points.push_back(p_origin);  // Arrow starts at origin
                direction_marker.points.push_back(arrow_end); // Arrow points along the direction
            }
                // // Publish the markers
                // marker_publisher_->publish(ray_marker);
                // marker_publisher_->publish(direction_marker);
            // }
            // Publish the markers
            //<Debug> 下面兩行要放在loop外面才publish不然會publish完又重新被弄調啥的, 反正會指出限三個ray
            marker_publisher_->publish(ray_marker);
            marker_publisher_->publish(direction_marker);
            RCLCPP_INFO(this->get_logger(), "Finished publishing all markers.");
            
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