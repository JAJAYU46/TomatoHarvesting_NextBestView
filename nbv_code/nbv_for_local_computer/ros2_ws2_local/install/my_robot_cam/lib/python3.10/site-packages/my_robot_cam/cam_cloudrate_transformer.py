#【結構】
#!/user/bin/env python3 

import rclpy #library for ROS2
from rclpy.node import Node
from geometry_msgs.msg import Twist #For publisher
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs  #For publisher

import time #for time_delay

#import keyboard #for keyboard輸入
# import keyboard
# #from pynput.keyboard import Key, Listener
# from pynput.keyboard import Key, Controller
from pynput import keyboard
# #========調整frame之間的轉換==========
# import tf2_ros  # For TF2 transformations
# # import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs  # For transforming PointCloud2 messages
# from tf2_ros import do_transform_cloud
from sensor_msgs_py import point_cloud2 as pc2 #for filter distance
import math

#=========== for TF ==================
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

import sys

INPUT_MODE=0 #1. gazebo big tomato 2. gazebo small tomato 3. realsense

class MyNode(Node): #construct Node class
    
    def __init__(self): #construct constructor
        super().__init__("cam_cloudrate_transformer") #set python_NodeName
        #Variable
        self.ReloadFlag=True #The flag for reloading: if T --> will reget frame from topic "/camera/camera/depth/color/points"
        self.Mode=0 #continue mode
        #【publisher】(publish to /cam/cloudrate_transformer)
        
         
        self.cloudrate_transformer_pub_=self.create_publisher(sensor_msgs.PointCloud2, "/cam/cloudrate_transformer", 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        #【subscriber】(subscribe from /camera/camera/depth/color/points)
        
        if(INPUT_MODE==1 or INPUT_MODE==2):
            self.PointCloud2_subscriber_=self.create_subscription(sensor_msgs.PointCloud2, "/camera/points", self.callback1, 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        else:
            self.PointCloud2_subscriber_=self.create_subscription(sensor_msgs.PointCloud2, "/camera/camera/depth/color/points", self.callback1, 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        # /camera/camera/depth/color/points
        #/camera/points （for gazebo) "/camera/camera/depth/color/points"(for realsense)

        # for 統一觀禮, 沒有做處理
        # <Debug>"/camera/image_raw" 不可以統一處理, 因為這個node還在做其他事情, 所以那個給detector 的image更新律也會因此便慢, 但是這個慢是沒必要的, 所以detector還是直接從源頭subscribe
        if(INPUT_MODE==1 or INPUT_MODE==2):
            self.Image_subscriber_=self.create_subscription(sensor_msgs.Image, "/camera/image_raw", self.callback_image_read, 10)
        else: 
            self.Image_subscriber_=self.create_subscription(sensor_msgs.Image, "/camera/camera/color/image_raw", self.callback_image_read, 10)
        self.Image_publisher_ = self.create_publisher(sensor_msgs.Image, '/cam/image_raw_original', 10)     # CHANGE
        
        
        
        
        
        
        self.get_logger().info("python_NodeName have been startedaa")
        
        # Time when the last message was received
        # self.last_received_time = time.time() ###
        # self.time_delay = 0 ### #設每0.5秒鐘才更新一次(也是需要讓它realtime）) #################也不能設三秒啥的, 這樣你作標會機器轉了但等於你point cloud publish在錯的角度上
        #因為其實對的作法要publish在現在這次偵測到的那個frame而不是直接publish 在此時刻的frame, 市要計算跟算frame的, 但這部份就可以先跳過
        # self.get_logger().info("now sample rate:"+str(self.time_delay)+" sec") ###
        
        #self.create_timer(1.0, self.callback1) #(time interval/ calling callback)
        #self.create_timer(1.0, self.callback_forSubscribe)
        #會定期偵測有沒有輸入e
        # # Keyboard listener
        # self.kb_listener = keyboard._listener(on_press=self.on_press)
        # self.kb_listener.start()
        #self.create_timer(1.0, self.callback_ModeType)
        # Keyboard listener

        # # TF2 Setup
        # # ==========================
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # #==========================
        # TF2 buffer and listener for frame transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)



        self.listener = keyboard.Listener(on_press=self.on_press)#,on_release=self.on_release)
        self.listener.start()
        # listener = keyboard.Listener(
        # on_press=self.on_press,
        # on_release=self.on_release)
        # listener.start()
        


    #for 2D image visualizasion
    def callback_image_read(self, msg_frame:sensor_msgs.Image):
        self.Image_publisher_.publish(msg_frame)

    # def callback_ModeType(self):
    #不知道為啥事subscriber那邊寫下之後，就不能用create_callback了，要下面這樣才行
    def callback1(self, msg:sensor_msgs.PointCloud2):# sensor_msgs.PointCloud2): #construct a callback
        # current_time = time.time() ###
        
        
        
        
        if self.Mode==0: #continue mode
            # #[加了frame的處理tf]
            # # Get the transformation from the camera frame to the odom frame
            # transform = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, rclpy.time.Time())
            # # Apply the transformation to the point cloud
            # transformed_cloud = do_transform_cloud(msg, transform)
            # # Publish the transformed point cloud
            # self.cloudrate_transformer_pub_.publish(transformed_cloud)

            #[下面是原本的但會有frame轉換錯方向的問題]
            # if current_time - self.last_received_time >= self.time_delay: ###
            try:
                # Get the transformation from the camera frame (or the frame you're working with)
                # transform: TransformStamped = self.tf_buffer.lookup_transform('camera_link_optical', msg.header.frame_id, rclpy.time.Time())
                            # Get the transformation at the time of point cloud capture
                # transform: TransformStamped = self.tf_buffer.lookup_transform(
                #     'camera_link_optical',    # Moving frame
                #     msg.header.frame_id,      # Original frame (typically fixed)
                #     msg.header.stamp)         # Time when the point cloud was captured
                pass
            except Exception as e:
                self.get_logger().error(f"Failed to get transform or process point cloud: {e}")

            filtered_points=self.distance_filter(msg) # <Note>發現這個函式超級慢 所以要處理一下frame的問題
            # msg.header=std_msgs.Header(frame_id='camera_link_optical', stamp=msg.header.stamp)#cam_frame_pcd(for realsense）#設定這個點雲圖會顯示到的frame ID
            msg.header=std_msgs.Header(frame_id='camera_link_optical')
            filtered_msg = pc2.create_cloud(msg.header, msg.fields, filtered_points)
            self.cloudrate_transformer_pub_.publish(filtered_msg)
            # self.cloudrate_transformer_pub_.publish(msg)
        elif self.Mode==1: #reload mode 
            if self.ReloadFlag==True: #key,
                #msg = sensor_msgs.PointCloud2()
                #if(True==True):
                
                #self.get_logger().info('r key pressed, reload pointcloud2')
                #self.get_logger().info(msg)
                #self.cloudrate_transformer_pub_.publish(msg)
                #self.callback_forPublish(self,msg)
                try:
                    # Get the transformation from the camera frame (or the frame you're working with)
                    # transform: TransformStamped = self.tf_buffer.lookup_transform('camera_link_optical', msg.header.frame_id, rclpy.time.Time())
                                # Get the transformation at the time of point cloud capture
                    # transform: TransformStamped = self.tf_buffer.lookup_transform(
                    #     'camera_link_optical',    # Moving frame
                    #     msg.header.frame_id,      # Original frame (typically fixed)
                    #     msg.header.stamp)         # Time when the point cloud was captured
                    pass
                except Exception as e:
                    self.get_logger().error(f"Failed to get transform or process point cloud: {e}")
                filtered_points=self.distance_filter(msg)    
                # msg.header=std_msgs.Header(frame_id='camera_link_optical', stamp=msg.header.stamp)#cam_frame_pcd設定這個點雲圖會顯示到的frame ID
                msg.header=std_msgs.Header(frame_id='camera_link_optical')#cam_frame_pcd設定這個點雲圖會顯示到的frame ID
                filtered_msg = pc2.create_cloud(msg.header, msg.fields, filtered_points)
                self.cloudrate_transformer_pub_.publish(filtered_msg)
                self.ReloadFlag=False
        # #============【Publish】===========
        # msg = Twist() #create a message object from the class twist
        # msg.linear.x = 2.0 #fill data of the message
        # msg.angular.z = 1.0
        # self.cmd_vel_pub_.publish(msg) #publish thing through this publisher topic
    # def callback_forPublish(self,msg): #construct a callback
    #     #============【Publish】===========
    #     msg = sensor_msgs.PointCloud2() #create a message object from the class twist
        
    #     #every time interval, do the code below to transfer pcd to point cloud
    #     self.data_pointCloud2=pcd_to_pointcloud2(self.points, self.colors, 'map') ##convertedPCD: variable contains the point cloud data
    #     #self.get_logger().info("LaLaLa")
    #     #self.get_logger().info(str(self.data_pointCloud2))
    #     self.pcd_publisher.publish(self.data_pointCloud2) #The default (fixed) frame in RViz is called 'map'
    #     #self.cmd_vel_pub_.publish(msg) #publish thing through this publisher topic
        
    # def callback_forSubscribe(self, msg: sensor_msgs.PointCloud2): #pose_callback #sent in a msg with message_type Pose的msg (msg as a Pose object)
        
        
    #     #============【Subscribe】============
    #     self.get_logger().info(str(msg.x))
    #     self.get_logger().info("("+str(msg.x)+", "+str(msg.y)+")")
    # r

    def distance_filter(self,msg2):
        filtered_points = []
        gen = pc2.read_points(msg2, skip_nans=True)
        int_data = list(gen)
        

        for point in int_data:
            x, y, z, rgb = point
            
            if not math.isinf(x) and not math.isinf(y) and not math.isinf(z):
                # self.get_logger().info("now z: "+str(z))
                if z<1: #z再依公尺以內的才要繼續傳下去處理（因為機器手必頂多採收一公尺以內的東西, 這樣可以加快運算速度）
                    # self.get_logger().info("now z: "+str(z))
                    filtered_points.append([x, y, z, rgb])
        return filtered_points
                    
    
    
    def on_press(self,key):
        #self.get_logger().info("key:"+str(key))
        #key_str = str(key)
        try: 
            # 因為在run node 的時候也沒半法, 所以都先關掉了
            # ========== 不可刪 ============
            # if key.char == 'r':#reload frame
            #     self.ReloadFlag=True
            #     self.get_logger().info("key be pressed(reload):"+str(key))
            # if key.char == 'c':#continued mode
            #     self.Mode=0
            #     self.get_logger().info("key be pressed(now mode):"+str(key))
            # if key.char == 'p':#loading mode
            #     self.Mode=1
            #     self.get_logger().info("key be pressed(nowload):"+str(key))
            # ==============================
            pass
        except: 
            pass
    # def on_release(self,key):
    #     self.ReloadFlag=False
    #     self.get_logger().info("key be released:"+str(key))


def main(args=None): #construct main function
    rclpy.init(args=args)

    global INPUT_MODE
    if len(sys.argv) > 1:
        INPUT_MODE = int(sys.argv[1])  # Get the argument from the command line
    else:
        INPUT_MODE = 3  # Default value  # default value if no args are provided  #1. gazebo big tomato 2. gazebo small tomato 3. realsense
    # run with 'ros2 run my_robot_nbv nbv_tompcd_filter 2'
    print("INPUT_MODE:", INPUT_MODE)



    node1 = MyNode() #node1=NodeClass: MyNode
    rclpy.spin(node1) #keep node alive until ctrl+C
    rclpy.shutdown()

if __name__=='__main__':
        main()	