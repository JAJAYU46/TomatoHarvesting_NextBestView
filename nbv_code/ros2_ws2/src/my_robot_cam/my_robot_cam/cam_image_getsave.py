#Detect Code
# import cv2

#================================
#for 2D visualizasion
#!/user/bin/env python3 
import rclpy #library for ROS2
from rclpy.node import Node
# from rclpy.clock import Clock
# from geometry_msgs.msg import Twist #For publisher
import sensor_msgs.msg as sensor_msgs
# import std_msgs.msg as std_msgs  #For publisher

import cv2
from cv_bridge import CvBridge, CvBridgeError
import threading


# import sys
# import os
# sys.path.append(os.path.dirname(os.path.realpath(__file__))) #approach adds the current directory where the script is located to the Python path.
# # for status controller topic
# from message_interfaces.msg import NodeStatus

# #!/user/bin/env python3 
# import rclpy #library for ROS2
# from rclpy.node import Node
# from rclpy.clock import Clock
# from geometry_msgs.msg import Twist #For publisher
# import sensor_msgs.msg as sensor_msgs
# import std_msgs.msg as std_msgs  #For publisher
# #import sensor_msgs.point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs_py import point_cloud2 as pc2

# import sensor_msgs.msg as sensor_msgs
# #================================
# #for 3D point cloud data
# import ctypes
# import struct
# import numpy as np
# import open3d as o3d

# #================================
# #for 2D visualizasion
# import cv2
# from cv_bridge import CvBridge, CvBridgeError
# #================================
# import math

# #===============================
# import sys
# import os
# sys.path.append(os.path.dirname(os.path.realpath(__file__))) #approach adds the current directory where the script is located to the Python path.

# #=========== for TF ==================
# from tf2_ros import Buffer, TransformListener
# from geometry_msgs.msg import TransformStamped 

# from scipy.spatial.transform import Rotation as R
#     # import numpy as np
# from geometry_msgs.msg import Point, PointStamped
# import tf2_geometry_msgs


# # custom message
# from message_interfaces.msg import BoundingBox    # CHANGE
# # from tutorial_interfaces.msg import Num    # CHANGE

# #To convert pointcloud2(for ROS2) to pointcloud(for open3d)
# # for saving output file
# import json
# INPUT_MODE=0



class MyNode(Node): #construct Node class
    def __init__(self): #construct constructor
        super().__init__("nbv_tompcd_filter") #set python_NodeName
        
        #[[相機參數]]
        # Camera parameters from Gazebo
        # horizontal_fov =  1.5009832 # in radians 1.089
        # image_width = 640  # in pixels
        # image_height = 480  # in pixels

        # # Calculate the focal length
        # self.fx = image_width / (2 * np.tan(horizontal_fov / 2))
        # self.fy = self.fx  # Assuming square pixels, so fx == fy

        # # Assume cx and cy are at the center of the image
        # self.cx = image_width / 2
        # self.cy = image_height / 2

        self.Image_subscriber_=self.create_subscription(sensor_msgs.Image, "/camera/camera/color/image_raw", self.callback_image_read, 10)


        self.frame1 = None # Initialize frame1 to None


        #[[For cv2]]
        self.bridge = CvBridge()

        # 啟動使用者互動執行緒
        threading.Thread(target=self.main_function, daemon=True).start()

        # self.timer = self.create_timer(1.0, self.callback1) #創見一個固定句其的定時器, 處理座標信息
  
    def main_function(self): #construct a callback
        while True:
            print("capture image? (y/n)")
            user_input = input().strip().lower()
            if user_input == 'y':
                if self.frame1 is not None:
                    cv2.imwrite("./src/Experiment/test20250805/test.jpg", cv2.resize(self.frame1, (int(self.frame1.shape[1]), int(self.frame1.shape[0]))))
                    print("Image saved successfully.")
                else:
                    print("No image captured yet.")
            elif user_input == 'q':
                print("Exiting...")
                break
            elif user_input == 'n':
                pass
                




    #for 2D image visualizasion
    def callback_image_read(self, msg_frame:sensor_msgs.Image):
        #img = cv2.imread('colar.jpg')
        # print(frame.shape)
        self.frame1 = self.bridge.imgmsg_to_cv2(msg_frame, "bgr8")
        # cv2.imwrite("./src/dataset/output_data/bounding_box/origin_frame.jpg", cv2.resize(frame1, (int(frame1.shape[1]), int(frame1.shape[0]))))
        


def main(args=None): #construct main funct沒有動ㄝ
    
    rclpy.init(args=args)

   
    node1 = MyNode() #node1=NodeClass: MyNode
    rclpy.spin(node1) #keep node alive until ctrl+C
    rclpy.shutdown()
    

if __name__=='__main__':
        main()	
