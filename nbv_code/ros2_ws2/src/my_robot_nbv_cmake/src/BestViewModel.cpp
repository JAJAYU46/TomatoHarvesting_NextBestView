


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

//for 把要publish的東東存起來留給之後的marker用
#include <queue>


// For custom topic status controller topic
#include "message_interfaces/msg/node_status.hpp" //在install>>message_interfaces...中看有沒有這個.hpp檔




#define DEBUG_MODE false

// 在SDF.h那裡
int INPUT_MODE1=2; //1. gazebo big tomato 2. gazebo small tomato 3. realsense


class BestViewModel {
public:
    //The data for outside to Get
    octomap::point3d BestCandidateView_point;
    int BestCandidateView_gain=0;
    float BestCandidateView_gain_percent=0.0;

    // //Data for last time
    // //The data for outside to Get
    // octomap::point3d LastBestCandidateView_point;
    // int LastBestCandidateView_gain=0;
    // float LastBestCandidateView_gain_percent=0.0;
    // bool isFinalNBVpoint = false;


    queue<octomap::point3d> markerQueue_origin; //這四個是互相對應的用來存放要傳給publish_any_ray marker的東東
    queue<std::vector<octomap::point3d>> markerQueue_endPoints;
    queue<int> markerQueue_id;
    queue<std::array<double, 3>> markerQueue_color;
    queue<double> markerQueue_xscale;


    //initialization
    BestViewModel(std::shared_ptr<open3d::geometry::PointCloud> pcdO3d_tomato, octomap::OcTree* octree, float candidateViews_radius, int candidateViews_num, int rays_num)
        : pcdO3d_tomato_(pcdO3d_tomato), octree_(octree), candidateViews_radius_(candidateViews_radius), candidateViews_num_(candidateViews_num) , rays_num_(rays_num) { //rays_num
        // marker_ID_handler[0]=0; //(ps: 之後是在跑各origin前++所以就會是從一開始)for green ray 100個candidate view 用id 1~100
        // marker_ID_handler[1]=candidateViews_num_; //for red ray id 101~200第二個candidate view要使用下一套ex若100個candidate view就會有100個red ray 100個 green ray 
        // marker_ID_handler[2]=2*candidateViews_num_; //for blue ray id 201~300
        // //0是留給nbv point的

        marker_ID_handler[0]=0; //(ps: 之後是在跑各origin前++所以就會是從一開始)for green ray 100個candidate view 用id 1~100
        marker_ID_handler[1]=100; //for red ray id 101~200第二個candidate view要使用下一套ex若100個candidate view就會有100個red ray 100個 green ray 
        marker_ID_handler[2]=200; //for blue ray id 201~300
        
        

    }

    // void ResetSceneForNextTomato(){
    //     BestCandidateView_gain=0;
    //     BestCandidateView_gain_percent=0.0;
    //     LastBestCandidateView_gain=0;
    //     LastBestCandidateView_gain_percent=0.0;
    //     isFinalNBVpoint = false;


    // }
    //FindNextBestView //做完會把這個scene(定by現在的pcdO3d_tomato_ 跟 octree_)的nbv傳承
    void CaculateNextBestView(){ //for a scene
        
        octomap::point3d Now_BestCandidateView_point;
        int Now_BestCandidateView_gain=0;
        //====== 1. 建立此scene的蕃茄的SDF model: ======
        //[從這裡建立SdfModel] sdf model就是件一個surface表格, 可判斷點在surface 內部還是外部一個model就是要處理的一個"ModelScene"
        SdfModel ModelScene(pcdO3d_tomato_, 0.2f, 5000); //這個pcd是要從另一個topic讀過來
        vector<float> modelCenter = ModelScene.GetModelCenter();
        octomap::point3d modelCenterOctomap_pcd = {modelCenter[0], modelCenter[1], modelCenter[2]};

        //====== 2. 建立Candidate views ====== (已Target tomato 的 center為中心（算by Sdf model), r=0.5為半徑製造上半球的random candidate views
        std::vector<octomap::point3d> origins_original = RandomPoint(modelCenterOctomap_pcd , candidateViews_num_, candidateViews_radius_); //中心座標, 點數, 長度
        std::vector<octomap::point3d> origins;
        octomap::point3d RobotArmBase(0.0,0.0,0.0); // 之後會被真正的base位置取代（應概就是0,0,0)車車是代表camera, end effector
        octomap::point3d RobotArmFrontAxisVec(1.0,0.0,0.0);
        
        cout<<"There are total "<<origins_original.size()<<" candidate view"<<endl;  
        //在這邊過濾點（因為candidate view 頂多100個所以這裡過濾不太影響時間複雜度)
        //也可以在上面製造的時候就過濾好, 但這裡邏輯是先隨機製造, 再過濾掉不能到達點
        // int origin_size = origins_original.size();
        // for (int i=0; i<origin_original_size;i++) {
        // int k=0;
        for (const auto& origin : origins_original){// && (int i=0; i<origin_size;i++) ){
            // if (k >= origin_size) {
            //     break; // Exit the loop if `i` exceeds `origin_size`
            // }
            // k++;
            
            
            // octomap::point3d origin = origins_original.pop_back();
            octomap::point3d vectorBO = origin-RobotArmBase;
            octomap::point3d vectorOT = modelCenterOctomap_pcd - origin;
            octomap::point3d vectorBT = modelCenterOctomap_pcd - RobotArmBase; //
            
            // calculate u project to v 
            // u : any
            // v : octomap::point3d RobotArmFrontAxisVec(1.0,0.0,0.0);
            double dotProductVV_RA_RA = RobotArmFrontAxisVec.dot(RobotArmFrontAxisVec);
            if (dotProductVV_RA_RA == 0) {
                std::cerr << "Error: Cannot project onto a zero vector." << std::endl;
                continue;
            }
            double dotProductUV_BO_RA = vectorBO.dot(RobotArmFrontAxisVec);
            double dotProductUV_OT_RA = vectorOT.dot(RobotArmFrontAxisVec);
            
            double scalarProjection_BO_RA = dotProductUV_BO_RA / dotProductVV_RA_RA;
            double scalarProjection_OT_RA = dotProductUV_OT_RA / dotProductVV_RA_RA;
            // Calculate the projection vector
            // octomap::point3d projection_BO_RA = RobotArmFrontAxisVec * scalarProjection_BO_RA;
            // octomap::point3d projection_OT_RA = RobotArmFrontAxisVec * scalarProjection_OT_RA;
            // 如果投影長度是負數的話
            // cout<<"OK1.......scalarProjection_OT_RA: "<<scalarProjection_OT_RA<<"scalarProjection_BO_RA: "<<scalarProjection_BO_RA<<endl;  
        
            if(scalarProjection_OT_RA<=0){ // 蕃茄背面不行
                // cout<<"Ok2...... "<<origins.size()<<" candidate views after back filtering"<<endl;  
        
                continue;
            }
            if(scalarProjection_BO_RA<=0){//手臂背面不行
                continue; //就不會做下面了, 會直接跳到下個origin
            }   
            
            origins.push_back(origin); //過審核的話再從後面丟進去
            

        }
        origins_original.clear();  // Clears all elements in the vector
        std::vector<octomap::point3d>().swap(origins_original);  // Releases the memory to shrink the capacity
        cout<<"There are total "<<origins.size()<<" candidate views after back filtering"<<endl;  
        
        //====== 3. 跑每個candidate view看每個candidate view的gain為多少
        for (const auto& origin : origins) {
            marker_ID_handler[0]++;
            marker_ID_handler[1]++;
            marker_ID_handler[2]++;
            //(for each candidate view(origin))
            
            //[3-1] Generate Random Rays (find random end points)
            // std::vector<octomap::point3d> directionsEndP = RandomPoint_Direction(origin, rays_num_, 0.1, modelCenterOctomap_pcd); //modelCenterOctomap_pcd其實不用太長0.5, 反正它只是個方向
            std::vector<octomap::point3d> directionsEndP = SystemedPoint_Direction(origin, rays_num_, 0.1, modelCenterOctomap_pcd); //modelCenterOctomap_pcd其實不用太長0.5, 反正它只是個方向
            
            std::vector<octomap::point3d> hitpointV; //紀錄被打到的grid的中心的座標們
            std::vector<octomap::point3d> directionsEndPV;

            //====== 4. 跑each ray看每個ray有沒有射到東西 ======
            cout<<"There are total "<<directionsEndP.size()<<" generated ray"<<endl;  
            for (const auto& directionEndP : directionsEndP) { //跑each ray

                //[4-1] 看這個ray射到哪個grid By Octomap castRay()
                octomap::point3d hitPoint; 
                octomap::point3d realdirection(directionEndP.x() - origin.x(), directionEndP.y() - origin.y(), directionEndP.z() - origin.z()); //<Debug8> 注意, casrRay是要傳進去"realdirection"'向量', 你現在RandomPointDirection算出來的是endpoint不是向量, 因為你publish_any_ray也是傳入一排的endpoint所以綠綠看起才是對的, 但實際計算用的藍藍是錯的
                bool hit = octree_->castRay(origin, realdirection, hitPoint, true);//<Debug8> //ray起點/ ray方向向量/ 打到的點座標要存到哪/ 是否ignore unknown point(不再octree node中的點)
                // RCLCPP_INFO(this->get_logger(), "Direction: (%f, %f, %f)", direction.x(), direction.y(), direction.z());

                //[4-2] 若有打到東西, 就存進hitpointV做後續處理
                if (hit) {
                    hitpointV.push_back(hitPoint);
                    directionsEndPV.push_back(directionEndP);
                    
                } else {
                    // RCLCPP_INFO(this->get_logger(), "No hit detected along the ray.");
                    directionsEndPV.push_back(directionEndP);
                    // directionsV.push_back(greendirection);
                }                                 

            }

            //====== 5. 跑each有hit到東西的ray是不是打到蕃茄 ====== (被hit的point 有沒有在蕃茄的sdf中)
            std::vector<Eigen::Vector3f> eigen_hitpoints = convertOctomapToEigenVector(hitpointV); //因為sdf model是吃vector的

            ModelScene.ComputeSDF(eigen_hitpoints);//有被打到的那些grid點, 丟進去sdf中看有沒有在surface裡面
            

            //====== 6. 計算gain ====== (用sdfmodel裡面的最後留有幾個點在sdf裡面, 點數就=此candidate view 的那些有打到蕃茄的rays的數量)
            array<int, 2> countInOut=ModelScene.ShowInPointCount();//會回傳現在有幾個點在surface內, 幾個在外的陣列
            int gain=countInOut[0]*1; //countInOut[0]為在內部的點數, 在內部的點數就存為gain
            
            if (DEBUG_MODE) {
                cout<<endl<<"============ Report  ==========="<<endl;
                cout<<"for Candidate View: ("<<origin.x()<<", "<<origin.y()<<", "<<origin.z() <<")"<<endl;
                cout<<"---------------------------------------"<<endl;
                cout<<"There are: ("<<countInOut[0]<<") points in the surface"<<endl;
                cout<<"There are: ("<<countInOut[1]<<") points out the surface"<<endl;
                cout<<"The gain is: "<<gain<<endl;
                cout<<"The gain percentage is: "<<(float(gain)/float(rays_num_))*100 <<"%"<<endl;
                // cout<<"The gain for candidate view at ("<<origin.x()<<", "<<origin.y()<<", "<<origin.z()<<") is "<<gain<<endl;
                cout<<"==============================================="<<endl;
            }
            //====== 7. 更新 best candidate view ====== (如果此點比之前的gain更大, 就更新best candidate view的點和gain)
            //To find best candidate view
            if(gain>Now_BestCandidateView_gain){
                Now_BestCandidateView_point=origin;
                Now_BestCandidateView_gain=gain;
                // publish_point_marker(origin.x(), origin.y(), origin.z(), 0.05, 1.0f, 1.0f, 0.0f);//scale color r g b
            }
            
            
            //====== 8. save for publish visualization marker ======
                        //要publish 綠色的那些ray, 下面是for 某個candidate view publish它的所有ray
            saveFor_publish_any_ray_marker(origin, directionsEndPV, marker_ID_handler[0], {0,1,0}, 0.003);

            //[save for red and blue ray marker]// 
            std::vector<Eigen::Vector3f> query_points_in_sdf = ModelScene.ShowInQueryPoints();
            std::vector<Eigen::Vector3f> query_points_out_sdf = ModelScene.ShowOutQueryPoints();
            // Call the conversion function
            std::vector<octomath::Vector3> converted_points_in_sdf = convertEigenToOctomap(query_points_in_sdf);
            std::vector<octomath::Vector3> converted_points_out_sdf = convertEigenToOctomap(query_points_out_sdf);
            
            //hit到蕃茄的ray要變成紅色 (hit到蕃茄的ray就是query_points_out_sdf)
            saveFor_publish_any_ray_marker(origin, converted_points_in_sdf, marker_ID_handler[1], {1,0,0}, 0.005);
            //hit到grid的ray要變成藍色 (hit到的就同進入sdf的query_points_in_sdf)
            // saveFor_publish_any_ray_marker(origin, converted_points_out_sdf, marker_ID_handler[2], {0,0,1}, 0.005);
            
            




            ModelScene.ResetCandidatePoint();//在要換一個candidate view前要做(對sdf model中的東東gain拉countIn point 數之類的reset(sdfmodel本身沒有reset))

        }
        // LastBestCandidateView_gain = BestCandidateView_gain;
        // LastBestCandidateView_point = BestCandidateView_point;
        // LastBestCandidateView_gain_percent = BestCandidateView_gain_percent;

        std::cout << "Now_BestCandidateView_gain: " << Now_BestCandidateView_gain << std::endl;
        std::cout << "rays_num_: " << rays_num_ << std::endl;

        if (rays_num_ == 0) {
            std::cerr << "Error: rays_num_ is zero, division not possible!" << std::endl;
        } else {
            BestCandidateView_gain_percent = float(Now_BestCandidateView_gain) / float(rays_num_);
            std::cout << "Calculated BestCandidateView_gain_percent: " << BestCandidateView_gain_percent << std::endl;
        }
        BestCandidateView_point = Now_BestCandidateView_point;
        BestCandidateView_gain = Now_BestCandidateView_gain;
        // BestCandidateView_gain_percent = float(Now_BestCandidateView_gain)/float(rays_num_);
        // BestCandidateView_gain_percent = float(Now_BestCandidateView_gain) / float(rays_num_);
        
        // if((BestCandidateView_gain_percent-LastBestCandidateView_gain_percent)<0.05){ //如果新的點比舊的點少於5% 那表示這就是最終的NBV點了
        //     BestCandidateView_point = LastBestCandidateView_point;
        //     BestCandidateView_gain = LastBestCandidateView_gain;
        //     BestCandidateView_gain_percent = LastBestCandidateView_gain_percent;
        //     isFinalNBVpoint = true;


        // }
        if (DEBUG_MODE) {
            cout<<"==============================================="<<endl;
            cout<<"The Best candidate view for this scene is at ("<<BestCandidateView_point.x()<<", "<<BestCandidateView_point.y()<<","<<BestCandidateView_point.z()<<") \nwith gain="<<BestCandidateView_gain<<endl;
            cout<<"===============================================";
        }
        if (DEBUG_MODE) {
            ModelScene.Visualize(); //現在就是會顯示最後一個點的狀況 //<Debug11>No 沒辦法, 因為在那之前最後一個點的point_就被clear了, 頂多放在candidate view那個裡面變成每點都有, 現在這樣一定是空的, 但可以當一個分段點拉
        }

    }


    
private:
    std::shared_ptr<open3d::geometry::PointCloud> pcdO3d_tomato_;
    octomap::OcTree* octree_;
    float candidateViews_radius_;
    int candidateViews_num_; //default要20
    int rays_num_; //default要50


    array<int, 3> marker_ID_handler = {0,0,0};
    //=================== 必要的function ================================
    //[Generate RandomPoint around a origin and return the end points vector]
    std::vector<octomap::point3d> RandomPoint(octomap::point3d center_, int pointNum, double radius_){
    // std::deque<octomap::point3d> RandomPoint(octomap::point3d center_, int pointNum, double radius_){
        if (DEBUG_MODE) {
            cout<<"generating "<<pointNum<<"candidate views..."<<endl;    
        }
        std::vector<octomap::point3d> RayEndPoints;
        // std::deque<octomap::point3d> origins; //因為要從前面pop從後面加

        std::random_device rd;
        std::mt19937 gen(rd());
        // std::uniform_real_distribution<> dist_phi(-M_PI/4, M_PI/4);//0, 2 * M_PI 0, M_PI: 只有前面
        std::uniform_real_distribution<> dist_phi(-M_PI, M_PI);//整圈////////////////////
        std::uniform_real_distribution<> dist_r(radius_, radius_);
        // std::uniform_real_distribution<> dist_theta(-1*M_PI, 1*M_PI); //如果是0的話就是z平面上
        // std::uniform_real_distribution<> dist_theta(0, 0.5);//只要上半球
        // std::uniform_real_distribution<> dist_cos_theta(0, 1.0); //<chatgpt2>
        std::uniform_real_distribution<> dist_cos_theta(-1.0, 1.0); //<chatgpt2>




        for (int i = 0; i < pointNum; ++i) {

            float phi = dist_phi(gen);
            float r = dist_r(gen);
            // float theta=acos(dist_theta(gen)); //<chatgpt2> 說這樣會不能uniform, 因為acos不是uniform 的, 所以改用上下面chatgpt2
            float theta = acos(dist_cos_theta(gen)); //<chatgpt2>

            
            float x = center_.x() + r * sin(theta)*cos(phi);
            float y = center_.y() + r * sin(theta)*sin(phi);
            float z = center_.z() + r * cos(theta);//0.05 ;//現在因為是用車車測試所以就用平面地板就好//center_.z() + r * cos(theta); /////////////////////////////////////

            // RayEndPoints.emplace_back(Eigen::Vector3f(x, y, z));
            RayEndPoints.push_back(octomap::point3d(x, y, z));
        }

        return RayEndPoints;

    }
    
    // std::vector<octomap::point3d> SystemedPoint_Direction(octomap::point3d center_, int pointNum, double radius_, octomap::point3d torwordP={1,0,0}){ //gazebo座標最前方是yx?？x?
    //     //RandomPoint_Direction 以center_ torwardP vector 為中心去建立那個ray
    //     std::vector<octomap::point3d> RayEndPoints;

    //     //==============================
    //     //  octomap::point3d torwordP={1,0,0};
    //     //現在中心是沿著x軸去generate 那個ray, 但是我希望是以center_ torwardP vector 為中心去建立那個ray, 往下什麼的也是, 所以要把這些end point做旋轉（因為往下也是, 所以要用旋轉的)
    //     Eigen::Vector3f direction(torwordP.x() - center_.x(), torwordP.y() - center_.y(), torwordP.z() - center_.z());
    //     // direction.normalize(); // Normalize the direction vector

    //     // Direction vector components
    //     float dx = torwordP.x() - center_.x();
    //     float dy = torwordP.y() - center_.y();
    //     float dz = torwordP.z() - center_.z();

    //     // Magnitude of the direction vector (r)
    //     float r = std::sqrt(dx * dx + dy * dy + dz * dz);
    //     float torwardP_phi = atan2(dy, dx);  // <Debug9> 加了綠綠的方向就正常了ㄟ好神奇喔<chatgpt> 說這樣可以generate整個circleatan2 handles full circular range
    //     float torwardP_theta = -asin(dz/r);
    //     //==============================
    //     //[structured ray] <待處理> 還沒改, 原本的random ray已經留在下方了
    //     int theta_steps=5, phi_steps=pointNum/5;
    //     float theta_min = -0.5, theta_max = 0.5;
    //     float phi_min = -M_PI / 4, phi_max = M_PI / 4;

    //     float theta_step = (theta_max - theta_min) / (pointNum - 1);
    //     float phi_step = (phi_max - phi_min) / (pointNum - 1);

    //     for (int i = 0; i < theta_steps; ++i) {
    //         float theta = acos(theta_min + i * theta_step) + torwardP_theta;

    //         for (int j = 0; j < phi_steps; ++j) {
    //             float phi = phi_min + j * phi_step + torwardP_phi;

    //             // Calculate Cartesian coordinates
    //             float x = center_.x() + radius_ * sin(theta) * cos(phi);
    //             float y = center_.y() + radius_ * sin(theta) * sin(phi);
    //             float z = center_.z() + radius_ * cos(theta);

    //             RayEndPoints.push_back(octomap::point3d(x, y, z));
    //         }
    //     }

    //     // for (int i = 0; i < pointNum; ++i) {
    //     //     float phi = dist_phi(gen)+torwardP_phi;
    //     //     float r = dist_r(gen);
    //     //     float theta=acos(dist_theta(gen))+torwardP_theta;
            
    //     //     float x = center_.x() + r * sin(theta)*cos(phi);
    //     //     float y = center_.y() + r * sin(theta)*sin(phi);
    //     //     float z = center_.z() + r * cos(theta);

    //     //     // RayEndPoints.emplace_back(Eigen::Vector3f(x, y, z));
    //     //     RayEndPoints.push_back(octomap::point3d(x, y, z));
    //     // }
        


    //     // std::random_device rd;
    //     // std::mt19937 gen(rd());
    //     // std::uniform_real_distribution<> dist_phi(-M_PI/4, M_PI/4);//0, 2 * M_PI 0, M_PI: 只有前面
    //     // // std::uniform_real_distribution<> dist_phi(0, 0);
    //     // // std::uniform_real_distribution<> dist_phi(-M_PI, M_PI);//整圈////////////////////
    //     // std::uniform_real_distribution<> dist_r(radius_, radius_);
    //     // // std::uniform_real_distribution<> dist_theta(-1*M_PI, 1*M_PI); //如果是0的話就是z平面上
    //     // std::uniform_real_distribution<> dist_theta(-0.5, 0.5);
    //     // // std::uniform_real_distribution<> dist_theta(0, 0);

    //     // for (int i = 0; i < pointNum; ++i) {
    //     //     float phi = dist_phi(gen)+torwardP_phi;
    //     //     float r = dist_r(gen);
    //     //     float theta=acos(dist_theta(gen))+torwardP_theta;
            
    //     //     float x = center_.x() + r * sin(theta)*cos(phi);
    //     //     float y = center_.y() + r * sin(theta)*sin(phi);
    //     //     float z = center_.z() + r * cos(theta);

    //     //     // RayEndPoints.emplace_back(Eigen::Vector3f(x, y, z));
    //     //     RayEndPoints.push_back(octomap::point3d(x, y, z));
    //     // }

    //     //=======================================
    //     //[random ray]
    //     // std::random_device rd;
    //     // std::mt19937 gen(rd());
    //     // std::uniform_real_distribution<> dist_phi(-M_PI/4, M_PI/4);//0, 2 * M_PI 0, M_PI: 只有前面
    //     // // std::uniform_real_distribution<> dist_phi(0, 0);
    //     // // std::uniform_real_distribution<> dist_phi(-M_PI, M_PI);//整圈////////////////////
    //     // std::uniform_real_distribution<> dist_r(radius_, radius_);
    //     // // std::uniform_real_distribution<> dist_theta(-1*M_PI, 1*M_PI); //如果是0的話就是z平面上
    //     // std::uniform_real_distribution<> dist_theta(-0.5, 0.5);
    //     // // std::uniform_real_distribution<> dist_theta(0, 0);
    //     // // RayEndPoints.push_back(octomap::point3d(center_.x(), center_.y(), center_.z()));//////////////////////////////////
    //     // for (int i = 0; i < pointNum; ++i) {
    //     //     RCLCPP_INFO(this->get_logger(), "%d-th ray", i);
    //     //     RCLCPP_INFO(this->get_logger(), " ");
    //     //     float phi = dist_phi(gen)+torwardP_phi;
    //     //     float r = dist_r(gen);
    //     //     float theta=acos(dist_theta(gen))+torwardP_theta;
            
    //     //     float x = center_.x() + r * sin(theta)*cos(phi);
    //     //     float y = center_.y() + r * sin(theta)*sin(phi);
    //     //     float z = center_.z() + r * cos(theta);

    //     //     // RayEndPoints.emplace_back(Eigen::Vector3f(x, y, z));
    //     //     RayEndPoints.push_back(octomap::point3d(x, y, z));
    //     // }
    //     //=============================================================
    //     //這邊製造的是ray end points 不是direction!！要剪掉自己才會變成direction!！!
    //     return RayEndPoints;

    // }


    std::vector<octomap::point3d> SystemedPoint_Direction(octomap::point3d center_, int pointNum, double radius_, octomap::point3d torwordP={1,0,0}){ //gazebo座標最前方是yx?？x?
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
        float torwardP_phi = atan2(dy, dx);  // <Debug9> 加了綠綠的方向就正常了ㄟ好神奇喔<chatgpt> 說這樣可以generate整個circleatan2 handles full circular range
        float torwardP_theta = -asin(dz/r);
        //==============================
        //[structured ray] <待處理> 還沒改, 原本的random ray已經留在下方了
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dist_phi(-M_PI/8, M_PI/8);//0, 2 * M_PI 0, M_PI: 只有前面
        // std::uniform_real_distribution<> dist_phi(0, 0);
        // std::uniform_real_distribution<> dist_phi(-M_PI, M_PI);//整圈////////////////////
        std::uniform_real_distribution<> dist_r(radius_, radius_);
        // std::uniform_real_distribution<> dist_theta(-1*M_PI, 1*M_PI); //如果是0的話就是z平面上
        std::uniform_real_distribution<> dist_theta(-0.2, 0.2); //(-0.5, 0.5)
        // std::uniform_real_distribution<> dist_theta(0, 0);

        // int theta_steps=5, phi_steps=pointNum/5;
        int theta_steps=sqrt(pointNum);
        int phi_steps=pointNum/theta_steps;
        float theta_min = -0.0, theta_max = 0.0;
        // float theta_min = -M_PI / 8, theta_max = M_PI / 8;
        float phi_min = 0.0, phi_max = 0.0;
        if(INPUT_MODE1==1){
            theta_min = -0.5;
            theta_max = 0.5;
            // float theta_min = -M_PI / 8, theta_max = M_PI / 8;
            phi_min = -M_PI / 8;
            phi_max = M_PI / 8;
        }else{
            theta_min = -0.5/3.0;
            theta_max = 0.5/3.0;
            // float theta_min = -M_PI / 8, theta_max = M_PI / 8;
            phi_min = -M_PI / 8/3.0;
            phi_max = M_PI / 8/3.0;

        }
        float theta_step = float(theta_max - theta_min) / float(theta_steps- 1);
        float phi_step = float(phi_max - phi_min) / float(phi_steps - 1);  //<Debug float/int = int!!>      

        for (int i = 0; i < theta_steps; ++i) {
            float theta = acos(theta_min + i * theta_step) + torwardP_theta;
            // float theta = (theta_min + i * theta_step) + torwardP_theta;

            for (int j = 0; j < phi_steps; ++j) {
                float phi = phi_min + j * phi_step + torwardP_phi;

                // Calculate Cartesian coordinates
                float x = center_.x() + radius_ * sin(theta) * cos(phi);
                float y = center_.y() + radius_ * sin(theta) * sin(phi);
                float z = center_.z() + radius_ * cos(theta);

                RayEndPoints.push_back(octomap::point3d(x, y, z));
            }
        }


        // for (int i = 0; i < pointNum; ++i) {
        //     float phi = dist_phi(gen)+torwardP_phi;
        //     float r = dist_r(gen);
        //     float theta=acos(dist_theta(gen))+torwardP_theta;
            
        //     float x = center_.x() + r * sin(theta)*cos(phi);
        //     float y = center_.y() + r * sin(theta)*sin(phi);
        //     float z = center_.z() + r * cos(theta);

        //     // RayEndPoints.emplace_back(Eigen::Vector3f(x, y, z));
        //     RayEndPoints.push_back(octomap::point3d(x, y, z));
        // }

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
        float torwardP_phi = atan2(dy, dx);  // <Debug9> 加了綠綠的方向就正常了ㄟ好神奇喔<chatgpt> 說這樣可以generate整個circleatan2 handles full circular range
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

        for (int i = 0; i < pointNum; ++i) {
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

    void saveFor_publish_any_ray_marker(const octomap::point3d &marker_origin, std::vector<octomap::point3d> &marker_endPoints, int marker_id=0,const std::array<double, 3> &marker_color={1,0,0}, const double marker_xscale=0.03) {
        markerQueue_origin.push(marker_origin);
        markerQueue_endPoints.push(marker_endPoints);
        markerQueue_id.push(marker_id);
        markerQueue_color.push(marker_color);
        markerQueue_xscale.push(marker_xscale);
    }

    


    //==================== 功能性function ================================
    //Show ＆ Display
    void ShowDebugMessage(string message){
        cout<<message<<endl;
    }

    //Function for converting data type
    // Helper function to convert octomath::Vector3 to Eigen::Vector3f
    std::vector<Eigen::Vector3f> convertOctomapToEigenVector(const std::vector<octomath::Vector3>& octomap_vec) {
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

    // PointCloud2toOpen3d(pointcloud2)：將讀近來的Tomato model pointcloud形式檔案變成open3d檔案


    // open3dToRos2PointCloud2()
    // <marker>
};

//================= ROS2 node class =================
class MyNode : public rclcpp::Node
{
    public:
        MyNode()
        : Node("nbvc_ray_generation"), count_(0), tf_buffer_(this->get_clock())//, tf_listener_(tf_buffer_) //node的名稱
        {
            //[tf listener] for subscribe to tomato topic 轉成 open3d point cloud時 change to the frame that align with octomap frame
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
            
            //[for做好icp tomato 的pcd]
            cloud_o3d_icpTomato = std::make_shared<open3d::geometry::PointCloud>();
            firstPCD_ready_flag = false;
            //[publisher]
            marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
            pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visualization_pcd", 10);
        
            //[subscriber]
            octomap_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
                "/octomap_binary", 10, std::bind(&MyNode::octomap_callback, this, std::placeholders::_1));
            pcd_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/nbv/tompcd_ICPonly", 10, std::bind(&MyNode::pointcloud_callback, this, std::placeholders::_1));

            
            // status controller topic //auto 就是讓compiler自己偵測data type, custom topic和其他classic topic不一樣, 要用auto
            status_publisher_ = this->create_publisher<message_interfaces::msg::NodeStatus>("/nbv/status_communicator", 10);
            status_subscription_ = this->create_subscription<message_interfaces::msg::NodeStatus>("/nbv/status_communicator", 10, std::bind(&MyNode::status_topic_callback, this, std::placeholders::_1));
            

        }

    private:
        // last nbv control
        octomap::point3d LastBestCandidateView_point;
        int LastBestCandidateView_gain=0;
        float LastBestCandidateView_gain_percent=0.0;
        bool isFinalNBVpoint = false;

        //====== 0. for status controller topic ======
        bool ready_for_next_iteration_msg_;
        bool is_moving_msg_;
        int target_box_id_msg_;
        int iteration_msg_;
        bool detection_done_msg_;
        bool icp_done_msg_;
        bool octomap_done_msg_ ;//= false; //防止第一次沒有資料的時候
        bool nbv_done_msg_;
        float nbv_point_x_msg_;
        float nbv_point_y_msg_;
        float nbv_point_z_msg_;
        bool is_final_result_msg_;

        bool finish_get_status_topic=false;
        rclcpp::Publisher<message_interfaces::msg::NodeStatus>::SharedPtr status_publisher_;
        rclcpp::Subscription<message_interfaces::msg::NodeStatus>::SharedPtr status_subscription_;


        // <Debug>好像都讀不到 <Debug> 結果釋出使化寫成local變數的問題======================
        void status_topic_callback(const message_interfaces::msg::NodeStatus::SharedPtr msg){
            // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->num);              // CHANGE
            // RCLCPP_INFO(this->get_logger(), "I heard: ");              // CHANGE
            // RCLCPP_INFO(this->get_logger(), "hahahahahahaha");

            ready_for_next_iteration_msg_ = msg->ready_for_next_iteration;
            is_moving_msg_ = msg->is_moving;
            target_box_id_msg_ = msg->target_box_id;

            iteration_msg_ = msg->iteration;
            detection_done_msg_ = msg->detection_done;
            icp_done_msg_ = msg->icp_done;
            octomap_done_msg_ = msg->octomap_done;
            nbv_done_msg_ = msg->nbv_done;
            nbv_point_x_msg_ = msg->nbv_point_x;
            nbv_point_y_msg_ = msg->nbv_point_y;
            nbv_point_z_msg_ = msg->nbv_point_z;
            is_final_result_msg_ = msg->is_final_result;

            finish_get_status_topic=true;
        }

        //====== 1. node private Variables ======
        size_t count_;

        //[for tf listener]
        tf2_ros::Buffer tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        //[for the tomato pcd] (subscribe icp完的topic pointcloud2 然後轉成 open3d pcd)
        std::shared_ptr<open3d::geometry::PointCloud> cloud_o3d_icpTomato; //for 餵進sdf model
        bool firstPCD_ready_flag;

        // Define the subscriber variable as a class member
        //[for publisher]
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
        
        //[for subscriber]
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_subscription_;
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription_; //寫在public的那些變數subscribe_啥的都要在這裡先定義好
        
        //====== 2. callback functions ====== 
        //當icp topic有新的東西的時候, 就要更新現在的cloud_o3d_icpTomato
        void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            // // Clear the previous data
            cloud_o3d_icpTomato->Clear();//<Debug> 物件用. access 一個pointer 裡面的物件才用->

            // open3d_cloud_ = ConvertPointCloud2ToOpen3D(msg);
            cloud_o3d_icpTomato = pointCloud2ToOpen3D(msg, "base_link", tf_buffer_);
            if (DEBUG_MODE) {
                RCLCPP_INFO(this->get_logger(), "Success Received a new point cloud with %lu points", cloud_o3d_icpTomato->points_.size());
            }
            // [for Debug] 可以不用publish回去, 只是用來檢查這樣有沒有抓到的工具而已
            // sensor_msgs::msg::PointCloud2 cloud_ros_icpTomato = open3dToRos2PointCloud2(*open3d_cloud_, "base_link");//msg->header.frame_id
            // pcd_publisher_->publish(cloud_ros_icpTomato);
            
            firstPCD_ready_flag=true;
        }

        void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) //const
        {   if(finish_get_status_topic == true){ //因為一直讀不到controller topic所以一直進不去這邊 ======================
                RCLCPP_INFO(this->get_logger(), "in get status topic");
                
                
                
                if(octomap_done_msg_==false and icp_done_msg_==true){//它前一個步驟也要好才可以 //只有false然後又接收到octomap的時候, 才要求一次nbv
                    if(iteration_msg_ == 0){ // 如果是第0次iteration, 表示是新的tomato, 就要重設nbv scene
                        // msg_status2.iteration = iteration_msg_+1;
                        LastBestCandidateView_gain=0;
                        LastBestCandidateView_gain_percent=0.0;
                    }
                    
                    
                    if(firstPCD_ready_flag==true){
                        RCLCPP_INFO(this->get_logger(), "Received octomap data!!");
                        octomap::OcTree* octree = NULL; // octomap::OcTree(0.01);
                        AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
                        //====== 1. convert ros2 octree into octomap octree ======
                        if (tree){
                            octree = static_cast<octomap::OcTree*>(tree); //用這個static才跑得動 但就要確定msg type要一樣
                            RCLCPP_INFO(this->get_logger(), "Success getting tree for this scene!");
                            if (octree){
                                //status controller topic ===============================
                                // if(octomap_done_msg_ == false){
                                    // 因為已經有了
                                    auto msg_status = message_interfaces::msg::NodeStatus();   
                                    msg_status.ready_for_next_iteration = ready_for_next_iteration_msg_;
                                    msg_status.is_moving = is_moving_msg_;
                                    msg_status.target_box_id = target_box_id_msg_; 
                                    msg_status.iteration = iteration_msg_;
                                    msg_status.detection_done = detection_done_msg_;
                                    msg_status.icp_done = icp_done_msg_;
                                    msg_status.octomap_done = true; //只幫octomap改這個
                                    msg_status.nbv_done = nbv_done_msg_;
                                    msg_status.nbv_point_x = nbv_point_x_msg_;
                                    msg_status.nbv_point_y = nbv_point_y_msg_;
                                    msg_status.nbv_point_z = nbv_point_z_msg_ ;
                                    msg_status.is_final_result = is_final_result_msg_;
                                    status_publisher_->publish(msg_status);
                                // }
                                // ==================================================== 
                                
                                
                                octree->write("src/dataset/data_octomap/octomap_before_inter.ot"); //AbstractOcTree 是一個octomap的通用格式, 可以自動處理color octomap或是non color octomap. 但它會是以指標的方式存（所以要得到他指到的member要用->而非. 這是c++中的語法) 
                                //<Note> octovis data/sample.ot 想要看.ot檔要在cd ros2_ws2下用這個(注意相對路徑)
                                //把estimated tomato 換成的octomap 和 環境 octomap融合
                                // open3d_cloud_
                                // Iterate through Open3D point cloud and add points to the Octree
                                for (const auto& point : cloud_o3d_icpTomato->points_) {
                                    octree->updateNode(octomap::point3d(point.x(), point.y(), point.z()), true); // Mark as occupied
                                }

                                // Update inner occupancy for the Octree
                                octree->updateInnerOccupancy();
                                octree->write("src/dataset/data_octomap/octomap_after_inter.ot");





                                RCLCPP_INFO(this->get_logger(),"Success getting octree map for this scene!");
                                
                                RCLCPP_INFO(this->get_logger(),"Map received (%zu nodes, %f m res), \n saving to src/dataset/data_octomap/octomap_from_orig.ot", octree->size(), octree->getResolution());
                                octree->write("src/dataset/data_octomap/octomap_from_orig.ot"); //AbstractOcTree 是一個octomap的通用格式, 可以自動處理color octomap或是non color octomap. 但它會是以指標的方式存（所以要得到他指到的member要用->而非. 這是c++中的語法) 
                                
                                RCLCPP_INFO(this->get_logger(),"Success saving octomap .ot, \nStart to calculate next best view for this scene...");
                                
                                float candidateViews_radius = 0.0;
                                //====== 2. Calculate nbv point ======
                                if(INPUT_MODE1==1){
                                    candidateViews_radius = 0.5; 
                                }else{
                                    candidateViews_radius = 0.32;
                                }
                                int candidateViews_num =50; //20
                                int rays_num = 64;//要是完全平方數最好   50; //50

                                BestViewModel NbvScene(cloud_o3d_icpTomato, octree, candidateViews_radius, candidateViews_num, rays_num);
                                



                                NbvScene.CaculateNextBestView();


                                //====== 3. publish visualization marker ====== (for publish visualization marker, 不停把markerQueue_xxx內的東西從第一個讀出 & publish ＆ pop 掉, 直到在queue中待publish的marker全部被publish完)

                                while (!NbvScene.markerQueue_origin.empty()) {
                                    // Access the front element
                                    if(DEBUG_MODE){
                                        cout << "Publishing marker for origin: " << NbvScene.markerQueue_origin.front() << endl;
                                    }
                                    // cout << "---------Publishing marker for origin: " << NbvScene.markerQueue_origin.front() <<"------------"<< endl;
                                    // cout << "Now size of NbvScene.markerQueue_origin: " << NbvScene.markerQueue_origin.size() << endl;
                                    // cout << "Now size of NbvScene.markerQueue_endPoints: " << NbvScene.markerQueue_endPoints.size() << endl;
                                    // cout << "Now size of NbvScene.markerQueue_id: " << NbvScene.markerQueue_id.size() << endl;
                                    // cout << "Now size of NbvScene.markerQueue_color: " << NbvScene.markerQueue_color.size() << endl;
                                    // cout << "Now size of NbvScene.markerQueue_xscale: " << NbvScene.markerQueue_xscale.size() << endl;
                                    
                                    // cout << "Now NbvScene.markerQueue_origin: " << NbvScene.markerQueue_origin.front() << endl;
                                    // // cout << "Now NbvScene.markerQueue_endPoints: " << NbvScene.markerQueue_endPoints.front() << endl;
                                    // cout << "Now NbvScene.markerQueue_id: " << NbvScene.markerQueue_id.front() << endl;
                                    // cout << "Now NbvScene.markerQueue_color: (" << NbvScene.markerQueue_color.front()[0] << ", " << NbvScene.markerQueue_color.front()[1] << ", " << NbvScene.markerQueue_color.front()[2]<<")" << endl;
                                    // cout << "Now NbvScene.markerQueue_xscale: " << NbvScene.markerQueue_xscale.front() << endl;

                                    publish_any_ray_marker(NbvScene.markerQueue_origin.front(), NbvScene.markerQueue_endPoints.front(), NbvScene.markerQueue_id.front(), NbvScene.markerQueue_color.front(), NbvScene.markerQueue_xscale.front());
                                    
                                    // Remove the front element
                                    NbvScene.markerQueue_origin.pop();
                                    NbvScene.markerQueue_endPoints.pop();
                                    NbvScene.markerQueue_id.pop();
                                    NbvScene.markerQueue_color.pop();
                                    NbvScene.markerQueue_xscale.pop();
                                    
                                }
                                
                                // LastBestCandidateView_gain = BestCandidateView_gain;
                                // LastBestCandidateView_point = BestCandidateView_point;
                                // LastBestCandidateView_gain_percent = BestCandidateView_gain_percent;
                                // BestCandidateView_point = Now_BestCandidateView_point;
                                // BestCandidateView_gain = Now_BestCandidateView_gain;
                                // BestCandidateView_gain_percent = float(Now_BestCandidateView_gain)/float(rays_num_);
                                RCLCPP_INFO(this->get_logger(), "NbvScene.BestCandidateView_gain_percent=%f", NbvScene.BestCandidateView_gain_percent);
                                RCLCPP_INFO(this->get_logger(), "NbvScene.BestCandidateView_gain=%d", NbvScene.BestCandidateView_gain);
                                
                                RCLCPP_INFO(this->get_logger(), "LastBestCandidateView_gain_percent=%f", LastBestCandidateView_gain_percent);
                                RCLCPP_INFO(this->get_logger(), "LastBestCandidateView_gain=%d", LastBestCandidateView_gain);
                                
                                if(((NbvScene.BestCandidateView_gain_percent-LastBestCandidateView_gain_percent)<0.05) && ((LastBestCandidateView_gain_percent)>0.0001)){ //如果新的點比舊的點少於5% 那表示這就是最終的NBV點了
                                    NbvScene.BestCandidateView_point = LastBestCandidateView_point;
                                    NbvScene.BestCandidateView_gain = LastBestCandidateView_gain;
                                    NbvScene.BestCandidateView_gain_percent = LastBestCandidateView_gain_percent;
                                    isFinalNBVpoint = true;


                                }
                                LastBestCandidateView_gain = NbvScene.BestCandidateView_gain;
                                LastBestCandidateView_point = NbvScene.BestCandidateView_point;
                                LastBestCandidateView_gain_percent = NbvScene.BestCandidateView_gain_percent;

                                //status controller topic ===============================
                                if(nbv_done_msg_ == false){// or is_moving_msg_ == false){ //等等只能publish一次因為要給的視同一個點, 不可以一直給不同的點 所以要接好//在下一步還沒完成之前都一直publish因為 
                                    auto msg_status2 = message_interfaces::msg::NodeStatus();   
                                    msg_status2.ready_for_next_iteration = ready_for_next_iteration_msg_;
                                    msg_status2.is_moving = is_moving_msg_;
                                    msg_status2.target_box_id = target_box_id_msg_;
                                    msg_status2.iteration = iteration_msg_;
                                    msg_status2.detection_done = detection_done_msg_;
                                    msg_status2.icp_done = icp_done_msg_;
                                    msg_status2.octomap_done = true; //只幫octomap改這個 現在也只有這個上次改過 預防沒更新到
                                    msg_status2.nbv_done = true;
                                    msg_status2.nbv_point_x = NbvScene.BestCandidateView_point.x();
                                    msg_status2.nbv_point_y = NbvScene.BestCandidateView_point.y();
                                    msg_status2.nbv_point_z = NbvScene.BestCandidateView_point.z();
                                    if(isFinalNBVpoint==true){ //如果已經是最終點了
                                        //就要把下面這個改true
                                        msg_status2.is_final_result = true;
                                        // msg_status2.iteration = 0; 其實不能在這裡控管, 要在controller那邊控管
                                        // 留在最一開始給topic控管 因為detection 可能是找到不同顆了 所以下次如果iteration = 0 待表示新的tomato 了
                                        
                                        // //重設nbv scene
                                        // NbvScene.ResetSceneForNextTomato();
                                    }else{
                                        msg_status2.is_final_result = is_final_result_msg_;//基本上會是false
                                        // msg_status2.iteration = iteration_msg_+1;
                                    }

                                    
                                    
                                    status_publisher_->publish(msg_status2);
                                }
                                // ==================================================== 
                                

                                //====== 4. Show the result of NBV point & its gain ======
                                RCLCPP_INFO(this->get_logger(), "===============================================");
                                RCLCPP_INFO(this->get_logger(), "The Best candidate view for this scene is at (%f, %f, %f) with gain percentage = %f", NbvScene.BestCandidateView_point.x(), NbvScene.BestCandidateView_point.y(), NbvScene.BestCandidateView_point.z(), NbvScene.BestCandidateView_gain_percent);
                                RCLCPP_INFO(this->get_logger(), "The Best candidate view for this scene is at (%f, %f, %f) with gain = %d", NbvScene.BestCandidateView_point.x(), NbvScene.BestCandidateView_point.y(), NbvScene.BestCandidateView_point.z(), NbvScene.BestCandidateView_gain);
                                
                                RCLCPP_INFO(this->get_logger(), "===============================================");
                                publish_point_marker(NbvScene.BestCandidateView_point.x(), NbvScene.BestCandidateView_point.y(), NbvScene.BestCandidateView_point.z(), 0.05, 1.0f, 1.0f, 0.0f);//scale color r g b
                                // for status controller topic
                                finish_get_status_topic=false;


                                
                            }else{
                                RCLCPP_ERROR(this->get_logger(),"Error reading OcTree from stream (octree fail)");
                            }

                        }else {
                            RCLCPP_ERROR(this->get_logger(), "Failed to convert Octomap message to OcTree (tree fail)");
                            
                        }
                    
                    }
                }   
            }    
        }





        //====== 3. helper functions ======
        // 3-1: Function to convert PointCloud2 <--> Open3D point cloud
        std::shared_ptr<open3d::geometry::PointCloud> pointCloud2ToOpen3D(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& ros_pc2_msg, const std::string& target_frame, tf2_ros::Buffer& tf_buffer) {

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

        sensor_msgs::msg::PointCloud2 open3dToRos2PointCloud2(const open3d::geometry::PointCloud& cloud, const std::string& frame_id = "map") {
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

        // 3-2: Function to publish ray markers
        void publish_any_ray_marker(const octomap::point3d &origin, std::vector<octomap::point3d> &endPoints, int id=0,const std::array<double, 3> &marker_color={1,0,0}, double x_scale=0.03) {
            
            if(INPUT_MODE1==1){
                x_scale=x_scale;
            }else{
                x_scale=x_scale/3.0;
            }
            
            visualization_msgs::msg::Marker any_ray_marker;
            
            any_ray_marker.header.frame_id = "base_link";   // Change if you have another frame map
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
            //<Debug10> 在rviz2那邊, marker的history設定要設成"keep all"!! 不然會變成指保留了後面幾次publish

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
            if(DEBUG_MODE){
                RCLCPP_INFO(this->get_logger(), "Finished publishing any_ray_marker.");
            }
            
        }


        void publish_point_marker(float x, float y, float z, float scale=0.05, float r=1, float g=0, float b=0, float a=1) {
            visualization_msgs::msg::Marker marker;

            // Set frame_id and timestamp
            marker.header.frame_id = "base_link";  // Make sure this frame exists in your tf tree
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
};


int main(int argc, char **argv)
{   //ROS2
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MyNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
    return 0;
}
