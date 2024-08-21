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


        #[[For ROS2 publisher and subscriber]]
        #【publisher】
        self.tompcd_filter_pub_=self.create_publisher(sensor_msgs.PointCloud2, "/nbv/tompcd_filter", 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        #【subscriber】
        self.PointCloud2_subscriber_=self.create_subscription(sensor_msgs.PointCloud2, "/cam/cloudrate_transformer", self.callback1, 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        #for image
        self.image_visual_pub_=self.create_publisher(sensor_msgs.Image, "/nbv/image_visual", 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        self.Image_subscriber_=self.create_subscription(sensor_msgs.Image, "/camera/image_raw", self.callback_image_visual, 10)

        self.get_logger().info("node 'nbv_tompcd_filter' have been started")
        #self.create_timer(1.0, self.callback1) #(time interval/ calling callback)
        
        
    def callback1(self, msg:sensor_msgs.PointCloud2): #construct a callback
        #============【Publish】===========
        #Create a new PointCloud2 message for the filtered data
        filtered_points = []    #想要留下的那些point data
        colors_pcd_open3d = [] #給open3d用的顏色
        # # Define lists to hold the points and colors
        # points = []
        # colors = []
        #===============================================
        gen = pc2.read_points(msg, skip_nans=True)
        int_data = list(gen)
        # for a in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z", "rgb")):
        # for a in gen:
        #=============================================

        #=============================================
        #     x, y, z, rgb = a
        #     self.get_logger().info("pointa"+str(a))

        
        min_u=1000
        min_v=1000
        max_u=0
        max_v=0
        

        for point in int_data:
            x, y, z, rgb = point
            #self.get_logger().info("point"+str(point))
            #test = point[3] 
            # cast float32 to int so that bitwise operations are possible
            if not math.isinf(x) and not math.isinf(y) and not math.isinf(z):
                s = struct.pack('>f' ,rgb)
                i = struct.unpack('>l',s)[0]
                # you can get back the float value by the inverse operations
                pack = ctypes.c_uint32(i).value
                r = (pack & 0x00FF0000)>> 16
                g = (pack & 0x0000FF00)>> 8
                b = (pack & 0x000000FF)
                # prints r,g,b values in the 0-255 range
                            # x,y,z can be retrieved from the x[0],x[1],x[2]
                #self.get_logger().info("r:"+str(r)+" g:"+str(g)+" b:"+str(b))
                # if r > 30 and g < 30 and b < 30:
                #如果在tomato box內而且是紅色的才要留下來
                
                #self.get_logger().info("self.TomatoBox_lu[0]:"+str(self.TomatoBox_lu[0])+"self.TomatoBox_rd[0]:"+str(self.TomatoBox_rd[0])+"self.TomatoBox_lu[1]:"+str(self.TomatoBox_lu[1])+"self.TomatoBox_rd[1]:"+str(self.TomatoBox_rd[1]))
                
                #[3D Point project to 2D pixel]
                # point_3d = [x, y, z]
                point_3d = [x, y, z]
                
                u, v = self.project_point_to_pixel(*point_3d)
                if r > 100 and g < 100 and b < 100:
                # if r < 255:
                # if b > 100 and r < 100 and g < 100:
                    
                    if min_u>u:
                        min_u=u
                    if min_v>v:
                        min_v=v
                    if max_u<u:
                        max_u=u
                    if max_v<v:
                        max_v=v
                    # self.get_logger().info("紅紅的")
                    # self.get_logger().info("x:"+str(x)+"y:"+str(y)+"z:"+str(z))
                    # self.get_logger().info("u"+str(u)+"v:"+str(v)+"z:"+str(z))
                    # filtered_points.append([x, y, z, rgb])
                    #<Debug>注意image pixel的座標是右x, 下y 然後現在publish camera的那個frame是就是xy都正常對到image的x,y, 然後z是指向前面
                    if self.TomatoBox_lu[0]<u and u<self.TomatoBox_rd[0]:
                        if self.TomatoBox_lu[1]<v and v<self.TomatoBox_rd[1]:
                                filtered_points.append([x, y, z, rgb])
                                colors_pcd_open3d.append([(r/255),(g/255),(b/255)])
                                #self.get_logger().info("in box")
            # xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
            # rgb = np.append(rgb,[[r,g,b]], axis = 0)
        
        #self.get_logger().info("min_u:"+str(min_u)+"min_v:"+str(min_v)+"max_u:"+str(max_u)+"max_v:"+str(max_v))
        # self.TomatoBox_lu[0]=min_u
        # self.TomatoBox_lu[1]=min_v
        # self.TomatoBox_rd[0]=max_u
        # self.TomatoBox_rd[1]=max_v
        self.get_logger().info("done")
        msg.header=std_msgs.Header(frame_id='camera_link_optical')#, stamp=Clock().now().to_msg())#cam_frame_pcd(for realsense）#設定這個點雲圖會顯示到的frame ID      
        filtered_msg = pc2.create_cloud(msg.header, msg.fields, filtered_points)
        # self.tompcd_filter_pub_.publish(filtered_msg)
        self.tompcd_filter_pub_.publish(filtered_msg)


        # Save the filtered points to a PCD file using Open3D
        point_cloud = o3d.geometry.PointCloud()
        points = np.array(filtered_points)[:, :3] #紀錄x, y, z
        #colors = np.array(filtered_points)[:, 3:6]  # RGB colors

        # # Ensure the colors are of type float64 and shape (N, 3)
        # colors = colors.astype(np.float64)
        # colors=int(colors/ 255.0)
        #self.get_logger().info("colors: "+str(colors_pcd_open3d))

        point_cloud.points = o3d.utility.Vector3dVector(points)
        point_cloud.colors = o3d.utility.Vector3dVector(colors_pcd_open3d)
        o3d.io.write_point_cloud("/home/jajayu/Documents/NextBestView/pointCloudDataPLY/copy_of_filtered_msg2.pcd",point_cloud)
        #==============================================


    #for 2D image visualizasion
    def callback_image_visual(self, msg_frame:sensor_msgs.Image):
        #img = cv2.imread('colar.jpg')
        # print(frame.shape)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg_frame, "bgr8")
        except CvBridgeError as e:
            print(e)
        (rows,cols,channels) = cv_image.shape
        #if cols > 60 and rows > 60 :
            # cv2.circle(cv_image, (240,50), 10, 255)
        # frame_wide_half=cv_image.shape[1]/2
        # frame_height_half=cv_image.shape[0]/2
        cv2.circle(cv_image, (int(cv_image.shape[1]/2), int(cv_image.shape[0]/2)), 10, 255)
        #假想蕃茄物件辨識框框左上：P(345, 255) 右上：Q(393, 305)
        cv2.rectangle(cv_image,self.TomatoBox_lu,self.TomatoBox_rd,(150, 0, 200),2)



        #<debug> 
        cv2.imshow("Image window", cv_image)
        #print(cv_image.shape)
        
        cv2.waitKey(3)

        try:
            self.image_visual_pub_.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
    
    #2D投影到3D
    def project_pixel_to_point(self, u, v, z):
        if z <= 0:
            return None  # Depth must be positive

        # Reconstruct the 3D point from the 2D pixel coordinates and the depth
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        return x, y, z
    
    #3D投影到2D
    def project_point_to_pixel(self, x, y, z):#project_point_to_pixel(self, x, y, z):
        if z == 0:
            return None  # Avoid division by zero

        # Project the 3D point onto the 2D image plane
        u = self.fx * (x / z) + self.cx
        v = self.fy * (y / z) + self.cy

        return int(u), int(v)
    
def main(args=None): #construct main function
    rclpy.init(args=args)
    node1 = MyNode() #node1=NodeClass: MyNode
    rclpy.spin(node1) #keep node alive until ctrl+C
    rclpy.shutdown()

if __name__=='__main__':
        main()	





#========================
        

        # Extract point cloud data
        # for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z", "rgb")):
        #     x, y, z, rgb = point
        #     self.get_logger().info("point"+str(point))
        #     # points.append([x, y, z])
        #     # colors.append(rgb)

        #     # Ensure rgb is treated as an integer
        #     rgb = int(rgb*pow(10,38))  # Convert rgb to integer if it's not already
            

        #     r = int((rgb >> 16) & 0xFF)
        #     g = int((rgb >> 8) & 0xFF)
        #     b = int(rgb & 0xFF)
        #     self.get_logger().info("r:"+str(r)+" g:"+str(g)+" b:"+str(b))
        #     # Filter points that are mostly red
        #     # if r > 150 and g < 100 and b < 100:
            # #if r > 255 and g < 255 and b < 255:
            # filtered_points.append([x, y, z, rgb])

        
        # msg.header=std_msgs.Header(frame_id='camera_link_optical')#, stamp=Clock().now().to_msg())#cam_frame_pcd(for realsense）#設定這個點雲圖會顯示到的frame ID      
        # filtered_msg = pc2.create_cloud(msg.header, msg.fields, filtered_points)
        # # self.tompcd_filter_pub_.publish(filtered_msg)
        # self.tompcd_filter_pub_.publish(filtered_msg)