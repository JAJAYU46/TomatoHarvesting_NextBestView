#Detect Code
# import cv2

import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__))) #approach adds the current directory where the script is located to the Python path.
from module_detect_tom import DetectTomato

#!/user/bin/env python3 
import rclpy #library for ROS2
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import Twist #For publisher
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs  #For publisher
#import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

import sensor_msgs.msg as sensor_msgs
#================================
#for 3D point cloud data
import ctypes
import struct
import numpy as np
import open3d as o3d

#================================
#for 2D visualizasion
import cv2
from cv_bridge import CvBridge, CvBridgeError
#================================
import math

#===============================
import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__))) #approach adds the current directory where the script is located to the Python path.
#Pesonal Module
from module_ICP import ICPoperation
from lib_cloud_conversion_between_Open3D_and_ROS_colorf4 import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d

#=========== for TF ==================
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped 

from scipy.spatial.transform import Rotation as R
    # import numpy as np
from geometry_msgs.msg import Point, PointStamped
import tf2_geometry_msgs


# custom message
from message_interfaces.msg import BoundingBox    # CHANGE
# from tutorial_interfaces.msg import Num    # CHANGE

#To convert pointcloud2(for ROS2) to pointcloud(for open3d)


class MyNode(Node): #construct Node class
    def __init__(self): #construct constructor
        super().__init__("nbv_tompcd_filter") #set python_NodeName
        
        #[[相機參數]]
        # Camera parameters from Gazebo
        horizontal_fov =  1.5009832 # in radians 1.089
        image_width = 640  # in pixels
        image_height = 480  # in pixels

        # Calculate the focal length
        self.fx = image_width / (2 * np.tan(horizontal_fov / 2))
        self.fy = self.fx  # Assuming square pixels, so fx == fy

        # Assume cx and cy are at the center of the image
        self.cx = image_width / 2
        self.cy = image_height / 2






        #[[For cv2]]
        self.bridge = CvBridge()

        #假想蕃茄物件辨識框框左上：P(345, 255) 右上：Q(393, 305)
        self.TomatoBox_lu=(330, 230)#(253,279)#(417,166)#(240,292)#(345, 255) #蕃茄辨識框框左上點
        self.TomatoBox_rd=(370, 290)#(387,419)#(640,317)#(407, 460) #蕃茄辨識框框右下點

        # 【subscriber】#照片之後應該要改成從cam那邊讀才對 filter那邊的也是
        self.Image_subscriber_=self.create_subscription(sensor_msgs.Image, "/camera/image_raw", self.callback_image_read, 10)

        self.isFirstFrame=True
        self.DetectionDoneFlag = True
        # [publish]
        self.bbox_publisher_ = self.create_publisher(BoundingBox, '/nbv/bounding_box_msg', 10)     # CHANGE
        #[[For ROS2 publisher and subscriber]]
        #【publisher】
        # self.tompcd_filter_pub_=self.create_publisher(sensor_msgs.PointCloud2, "/nbv/tompcd_filter", 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        # self.tompcd_ICP_pub_=self.create_publisher(sensor_msgs.PointCloud2, "/nbv/tompcd_ICP", 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        # self.tompcd_ICPonly_pub_=self.create_publisher(sensor_msgs.PointCloud2, "/nbv/tompcd_ICPonly", 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        
        # #【subscriber】
        # self.PointCloud2_subscriber_=self.create_subscription(sensor_msgs.PointCloud2, "/cam/cloudrate_transformer", self.callback1, 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        # #for image
        # self.image_visual_pub_=self.create_publisher(sensor_msgs.Image, "/nbv/image_visual", 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        # self.Image_subscriber_=self.create_subscription(sensor_msgs.Image, "/camera/image_raw", self.callback_image_visual, 10)

        # self.get_logger().info("node 'nbv_tompcd_filter' have been started")
        # #self.create_timer(1.0, self.callback1) #(time interval/ calling callback)
        # # TF2 buffer and listener for frame transformations
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)
        # # self.timer = self.create_timer(1.0, self.on_timer) #創見一個固定句其的定時器, 處理座標信息

        
    def callback1(self, msg:sensor_msgs.PointCloud2): #construct a callback
        
        a=0
                




    #for 2D image visualizasion
    def callback_image_read(self, msg_frame:sensor_msgs.Image):
        #img = cv2.imread('colar.jpg')
        # print(frame.shape)
        try:
            frame1 = self.bridge.imgmsg_to_cv2(msg_frame, "bgr8")
        except CvBridgeError as e:
            print(e)
        (rows,cols,channels) = frame1.shape

        # 只要subscribe到照片, 就用一次object detection
        # #Load image
        # video_path = "dataset/data_tomatoVideo_forDetection/tomato_video2.mp4"  # Path to your video
        # cap = cv2.VideoCapture(video_path)

        
        resizeMag=1
        # #main
        # ret, frame1 = cap.read()
        # if not ret:
        #     print("End of video.")
        if(self.isFirstFrame==True): 
            # 只有最開始的frame要建立一個新的物件
            frame1_resized = cv2.resize(frame1, (int(frame1.shape[1]*resizeMag), int(frame1.shape[0]*resizeMag)))
            # # #initialize
            
            self.MyTomatoDetector = DetectTomato (frame1_resized, resizeMag) #給一個初始的圖片
            self.MyTomatoDetector.changeGetNewTargetTomato(True) 
            self.isFirstFrame = False
        
        # # MyTomatoDetector.changeGetNewTargetTomato(True) 
        # # TargetBox, image = MyTomatoDetector.DetectTomato(frame1_resized) #先得到初始的ID
        # # MyTomatoDetector.changeGetNewTargetTomato(False) 

        # while(True): 
        #     #main
        #     ret, frame1 = cap.read()
        #     if not ret:
        #         print("End of video.")
        #         break

        # <Debug1.1> 不可以這樣, 因為那個追蹤是線性的, 是要追蹤的, 你這樣它會找不到目標的那顆target 變成不publish東西
        # 之後可以加個, 如果追蹤的東東不見了, 就把最上面那個當成最新的目標
        # if(self.DetectionDoneFlag==True):# 因為yolo很花時間, 所以現在就是如果上一次的yolo還沒結束, 就不要處理新的照片, 新的msg frame就不處理（之後會變成, 在手必移動到新的點完成之前, 都不做處理）
        if(True): # <Debug1.1> 好像就算這樣也沒用, 因為之前影片就算lag, 它基本上還是線性的, 但這個frame在處理完之後才收msg, 是會掉包的, 所以不是線性的 所以這樣的話, 現在改成如果目標消失了, 就已最高的當目標
            self.DetectionDoneFlag=False
            frame1_resized = cv2.resize(frame1, (int(frame1.shape[1]*resizeMag), int(frame1.shape[0]*resizeMag)))

            TargetBox, image = self.MyTomatoDetector.DetectTomato(frame1_resized) #先得到初始的ID
            self.DetectionDoneFlag=True
            if TargetBox is None:
                print("No tomato detected.")
            else: 
                msg_box = BoundingBox()
                msg_box.lu_x = TargetBox[0]
                msg_box.lu_y = TargetBox[1]
                msg_box.rd_x = TargetBox[2]
                msg_box.rd_y = TargetBox[3]
                self.bbox_publisher_.publish(msg_box)
                # self.get_logger().info('Publishing: "%d"' % msg_box.lu_x) 
            cv2.imshow('Tomato Image', image)
            # cv2.imshow('frame1', frame1)
            cv2.waitKey(1)














        # #if cols > 60 and rows > 60 :
        #     # cv2.circle(cv_image, (240,50), 10, 255)
        # # frame_wide_half=cv_image.shape[1]/2
        # # frame_height_half=cv_image.shape[0]/2
        # cv2.circle(cv_image, (int(cv_image.shape[1]/2), int(cv_image.shape[0]/2)), 10, 255)
        # #假想蕃茄物件辨識框框左上：P(345, 255) 右上：Q(393, 305)
        # cv2.rectangle(cv_image,self.TomatoBox_lu,self.TomatoBox_rd,(150, 0, 200),2)



        # #<debug> 
        # cv2.imshow("Image window", cv_image)
        # #print(cv_image.shape)
        
        # cv2.waitKey(3)

        # try:
        #     self.image_visual_pub_.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # except CvBridgeError as e:
        #     print(e)
    


def main(args=None): #construct main funct沒有動ㄝ
    
    # if not __debug__:
    #     # Your main code here
    #     print("Running in optimized mode")
    #     # Proceed with normal node logic
    # else:
    #     # Relaunch the script in optimized mode
    #     script_path = os.path.realpath(__file__)
    #     os.execv(sys.executable, [sys.executable, '-O', script_path] + sys.argv[1:])
    # #===============================================================
    
    rclpy.init(args=args)
    node1 = MyNode() #node1=NodeClass: MyNode
    rclpy.spin(node1) #keep node alive until ctrl+C
    rclpy.shutdown()
    

if __name__=='__main__':
        main()	







# =============================================

# if __name__ == '__main__':
#     #Load image
#     video_path = "dataset/data_tomatoVideo_forDetection/tomato_video2.mp4"  # Path to your video
#     cap = cv2.VideoCapture(video_path)

#     resizeMag=1/3 
#     #main
#     ret, frame1 = cap.read()
#     if not ret:
#         print("End of video.")
    
#     frame1_resized = cv2.resize(frame1, (int(frame1.shape[1]*resizeMag), int(frame1.shape[0]*resizeMag)))
#     #initialize
#     MyTomatoDetector = DetectTomato (frame1_resized, resizeMag) #給一個初始的圖片
#     MyTomatoDetector.changeGetNewTargetTomato(True) 
#     # TargetBox, image = MyTomatoDetector.DetectTomato(frame1_resized) #先得到初始的ID
#     # MyTomatoDetector.changeGetNewTargetTomato(False) 

#     while(True): 
#         #main
#         ret, frame1 = cap.read()
#         if not ret:
#             print("End of video.")
#             break
#         frame1_resized = cv2.resize(frame1, (int(frame1.shape[1]*resizeMag), int(frame1.shape[0]*resizeMag)))

#         TargetBox, image = MyTomatoDetector.DetectTomato(frame1_resized) #先得到初始的ID

#         if TargetBox is None:
#             print("No tomato detected.")
#         cv2.imshow('Tomato Image', image)
#         cv2.waitKey(1)





