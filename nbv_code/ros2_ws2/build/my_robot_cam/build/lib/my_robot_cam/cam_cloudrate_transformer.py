#【結構】
#!/user/bin/env python3 

import rclpy #library for ROS2
from rclpy.node import Node
from geometry_msgs.msg import Twist #For publisher
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs  #For publisher
#import keyboard #for keyboard輸入
# import keyboard
# #from pynput.keyboard import Key, Listener
# from pynput.keyboard import Key, Controller
from pynput import keyboard
# #========調整frame之間的轉換==========
# import tf2_ros  # For TF2 transformations
# # import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs  # For transforming PointCloud2 messages
# from tf2_ros import do_transform_cloud


class MyNode(Node): #construct Node class
    
    def __init__(self): #construct constructor
        super().__init__("cam_cloudrate_transformer") #set python_NodeName
        #Variable
        self.ReloadFlag=True #The flag for reloading: if T --> will reget frame from topic "/camera/camera/depth/color/points"
        self.Mode=0 #continue mode
        #【publisher】(publish to /cam/cloudrate_transformer)
        self.cloudrate_transformer_pub_=self.create_publisher(sensor_msgs.PointCloud2, "/cam/cloudrate_transformer", 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        #【subscriber】(subscribe from /camera/camera/depth/color/points)
        self.PointCloud2_subscriber_=self.create_subscription(sensor_msgs.PointCloud2, "/camera/points", self.callback1, 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        #/camera/points （for gazebo) "/camera/camera/depth/color/points"(for realsense)
        self.get_logger().info("python_NodeName have been started")
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



        self.listener = keyboard.Listener(on_press=self.on_press)#,on_release=self.on_release)
        self.listener.start()
        # listener = keyboard.Listener(
        # on_press=self.on_press,
        # on_release=self.on_release)
        # listener.start()
        
    # def callback_ModeType(self):
    #不知道為啥事subscriber那邊寫下之後，就不能用create_callback了，要下面這樣才行
    def callback1(self, msg:sensor_msgs.PointCloud2):# sensor_msgs.PointCloud2): #construct a callback
        if self.Mode==0: #continue mode
            # #[加了frame的處理tf]
            # # Get the transformation from the camera frame to the odom frame
            # transform = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, rclpy.time.Time())
            # # Apply the transformation to the point cloud
            # transformed_cloud = do_transform_cloud(msg, transform)
            # # Publish the transformed point cloud
            # self.cloudrate_transformer_pub_.publish(transformed_cloud)

            #[下面是原本的但會有frame轉換錯方向的問題]
            msg.header=std_msgs.Header(frame_id='camera_link_optical')#cam_frame_pcd(for realsense）#設定這個點雲圖會顯示到的frame ID
            self.cloudrate_transformer_pub_.publish(msg)
        elif self.Mode==1: #reload mode 
            if self.ReloadFlag==True: #key,
                #msg = sensor_msgs.PointCloud2()
                #if(True==True):
                
                #self.get_logger().info('r key pressed, reload pointcloud2')
                #self.get_logger().info(msg)
                #self.cloudrate_transformer_pub_.publish(msg)
                #self.callback_forPublish(self,msg)
                msg.header=std_msgs.Header(frame_id='camera_link_optical')#cam_frame_pcd設定這個點雲圖會顯示到的frame ID
                self.cloudrate_transformer_pub_.publish(msg)
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
    def on_press(self,key):
        #self.get_logger().info("key:"+str(key))
        #key_str = str(key)
        try: 
            if key.char == 'r':#reload frame
                self.ReloadFlag=True
                self.get_logger().info("key be pressed(reload):"+str(key))
            if key.char == 'c':#continued mode
                self.Mode=0
                self.get_logger().info("key be pressed(now mode):"+str(key))
            if key.char == 'p':#loading mode
                self.Mode=1
                self.get_logger().info("key be pressed(nowload):"+str(key))
        except: 
            pass
    # def on_release(self,key):
    #     self.ReloadFlag=False
    #     self.get_logger().info("key be released:"+str(key))


def main(args=None): #construct main function
    rclpy.init(args=args)
    node1 = MyNode() #node1=NodeClass: MyNode
    rclpy.spin(node1) #keep node alive until ctrl+C
    rclpy.shutdown()

if __name__=='__main__':
        main()	