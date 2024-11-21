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
        self.tompcd_ICP_pub_=self.create_publisher(sensor_msgs.PointCloud2, "/nbv/tompcd_ICP", 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        self.tompcd_ICPonly_pub_=self.create_publisher(sensor_msgs.PointCloud2, "/nbv/tompcd_ICPonly", 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        
        #【subscriber】
        self.PointCloud2_subscriber_=self.create_subscription(sensor_msgs.PointCloud2, "/cam/cloudrate_transformer", self.callback1, 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        #for image
        self.image_visual_pub_=self.create_publisher(sensor_msgs.Image, "/nbv/image_visual", 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        self.Image_subscriber_=self.create_subscription(sensor_msgs.Image, "/camera/image_raw", self.callback_image_visual, 10)

        self.get_logger().info("node 'nbv_tompcd_filter' have been started")
        #self.create_timer(1.0, self.callback1) #(time interval/ calling callback)
        # TF2 buffer and listener for frame transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.timer = self.create_timer(1.0, self.on_timer) #創見一個固定句其的定時器, 處理座標信息

        
    def callback1(self, msg:sensor_msgs.PointCloud2): #construct a callback
        
        TransformStamped_before_ready=False
        try: #街收到新data後除非transform有成功讀到, 才繼續往下做
            # TransformStamped_before = self.tf_buffer.lookup_transform( #先把它轉成odom的座標用的TransformStamped
            # 'odom', # target frame Moving frame #也就是現在的'camera_link_optical',    
            # msg.header.frame_id, # world frame Original frame (typically fixed)（作為固定的參考, 才知道轉了多少)
            # rclpy.time.Time())#msg.header.stamp)         # Time when the point cloud was captured
            TransformStamped = self.tf_buffer.lookup_transform( #先把它轉成odom的座標用的TransformStamped
            'odom', # target frame Moving frame #也就是現在的'camera_link_optical',    
            msg.header.frame_id, # world frame Original frame (typically fixed)（作為固定的參考, 才知道轉了多少)
            msg.header.stamp)#rclpy.time.Time())#msg.header.stamp)         # Time when the point cloud was captured


            TransformStamped_before_ready=True
        except Exception as ex:
            self.get_logger().error(f'Could not transform frame because{ex}')
            TransformStamped_before_ready=False
            pass
        if(TransformStamped_before_ready==True): #好了才開始做下面


            
            
            
            #============【Publish】===========
            #Create a new PointCloud2 message for the filtered data
            filtered_points = []    #想要留下的那些point data
            # points_open3d = []
            colors_pcd_open3d = [] #給open3d用的顏色
            # # Define lists to hold the points and colors
            # points = []
            # colors = []
            #===============================================
            if __debug__:
                self.get_logger().info("========================")
                self.get_logger().info("new frame start process")
                self.get_logger().info("========================")
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
                                    # points_open3d.append([(r/255),(g/255),(b/255)])
                                    colors_pcd_open3d.append([(r/255),(g/255),(b/255)])
                                    # self.get_logger().info("in box")
                # xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
                # rgb = np.append(rgb,[[r,g,b]], axis = 0)
            
            #self.get_logger().info("min_u:"+str(min_u)+"min_v:"+str(min_v)+"max_u:"+str(max_u)+"max_v:"+str(max_v))
            # self.TomatoBox_lu[0]=min_u
            # self.TomatoBox_lu[1]=min_v
            # self.TomatoBox_rd[0]=max_u
            # self.TomatoBox_rd[1]=max_v
            if __debug__:
                self.get_logger().info("done filtering")
            msg.header=std_msgs.Header(frame_id='camera_link_optical')#, stamp=Clock().now().to_msg())#cam_frame_pcd(for realsense）#設定這個點雲圖會顯示到的frame ID      
            # msg.header=std_msgs.Header(frame_id='camera_link_optical', stamp=msg.header.stamp)
            filtered_msg = pc2.create_cloud(msg.header, msg.fields, filtered_points)
            # self.get_logger().info(f"filtered_points: {filtered_points}")
            # self.tompcd_filter_pub_.publish(filtered_msg)
            self.tompcd_filter_pub_.publish(filtered_msg)


            # Save the filtered points to a PCD file using Open3D
            point_cloud = o3d.geometry.PointCloud()
            if filtered_points: #如果有filter到東西(is not empty)再做下面的icp跟publish
                points = np.array(filtered_points)[:, :3] #紀錄x, y, z
                #colors = np.array(filtered_points)[:, 3:6]  # RGB colors

                # # Ensure the colors are of type float64 and shape (N, 3)
                # colors = colors.astype(np.float64)
                # colors=int(colors/ 255.0)
                #self.get_logger().info("colors: "+str(colors_pcd_open3d))

                point_cloud.points = o3d.utility.Vector3dVector(points)
                point_cloud.colors = o3d.utility.Vector3dVector(colors_pcd_open3d)
                
                source_path="/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/ros2_ws2/src/dataset/data_pcd/TomatoPlant_size_modified_only1tomato_onlyRed.ply"
                source = o3d.io.read_point_cloud(source_path.format(3)) #demo_pcds.paths[0])
                # print(np.asarray(pcd.points))
                #for debug
                # source_np=np.array(source)
                # self.get_logger().info(f'source_np: {source_np}')
                # self.get_logger().info(f'Data_type of source_np: {source_np.dtype}')

                target = point_cloud

                # try:
                if(1==1):
                    self.get_logger().info("start ICP")
                    result_trans_final=ICPoperation(source,target)
                    if result_trans_final is not None: #才做下面的事情
                        source.paint_uniform_color([1, 0.706, 0])
                        source.transform(result_trans_final.transformation)

                        #debug#
                        # # Convert to numpy array and print the data type of each component
                        # source_points = np.asarray(source.points)  # Get points as a numpy array
                        # source_colors = np.asarray(source.colors)  # Get colors as a numpy array (if available)
                        
                        # # Print out the data and its type
                        # print(f"Points shape: {source_points.shape}, dtype: {source_points.dtype}")
                        # print(f"First point: {source_points[0]}")  # Example of accessing a point

                        # if source_colors.size > 0:
                        #     print(f"Colors shape: {source_colors.shape}, dtype: {source_colors.dtype}")
                        #     print(f"First color: {source_colors[0]}")  # Example of accessing a color
                        # else:
                        #     print("No colors available in the point cloud.")
                        ##





                        self.get_logger().info("done registering, start integrating two point cloud")
                        source_pointcloud2 = convertCloudFromOpen3dToRos(source, "camera_link_optical")
                        self.get_logger().info("convertCloudFromOpen3dToRos done")
                        # self.tompcd_ICPonly_pub_.publish(source_pointcloud2) #pc_array = np.array(list(pc_data)) //只publish那個轉好的model給之後算sdf用
                        
                        #把ICP data和原來的pointcloud2 data組合起來, 因為之後要給octomap算 弄不出來...20240825
                        # new_point = list(pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True))
                        # new_point = np.array(list(pc2.read_points(source_pointcloud2, field_names=("x", "y", "z", "rgb"))))
                        #20240905
                        whole_points_np = []
                        # gen = pc2.read_points(msg, skip_nans=True)
                        # int_data = list(gen)
                        msg_points_np = np.array(list(pc2.read_points(msg, skip_nans=True)))
                        source_points_np = np.array(list(pc2.read_points(source_pointcloud2, skip_nans=True)))
                        
                        # source_pointcloud2_headerstamped= pc2.create_cloud(msg.header, msg.fields, source_points_np.tolist())
                        # self.tompcd_ICPonly_pub_.publish(source_pointcloud2_headerstamped)
                        # if __debug__:
                        self.get_logger().info(f'Shape of msg_points_np.shape: {msg_points_np.shape}')
                        self.get_logger().info(f'Shape of source_points_np.shape: {(source_points_np.shape)}')
                        # # self.get_logger().info(f'Array of msg_points_np: {msg_points_np}')
                        # self.get_logger().info(f'Data_type of msg_points_np: {msg_points_np.dtype}')
                        # self.get_logger().info(f'Array of source_points_np: {(source_points_np)}')
                        # self.get_logger().info(f'Data_type of source_points_np: {(source_points_np.dtype)}')

                        whole_points_np = np.append(msg_points_np, source_points_np, axis=0)#往側邊append
                        # self.get_logger().info(f'Array of whole_points_np: {(whole_points_np)}')
                        if __debug__:
                            self.get_logger().info(f'Shape of whole_points_np: {(whole_points_np.shape)}')
                        # self.get_logger().info(f'msg.fields: {(msg.fields)}')
                        # self.get_logger().info(f'filter_points: {(filtered_points)}')                    self.get_logger().info("ok0")
                        # source_pointcloud2_transformed_points_np = self.transform_pointcloud_to_np(source_points_np, TransformStamped)
                        
                        # TransformStamped = self.tf_buffer.lookup_transform( #先把它轉成odom的座標用的TransformStamped
                        #     msg.header.frame_id, # target frame Moving frame #也就是現在的'camera_link_optical',    
                        #     rclpy.time.Time() , # world frame Original frame (typically fixed)（作為固定的參考, 才知道轉了多少)
                        #     msg.header.frame_id, #現在是camera link
                        #     msg.header.stamp,
                        #     'odom',
                        #     rclpy.duration.Duration(seconds=0)  # Correct way to create a Duration object
                        #     )         # Time when the point cloud was captured



                        TransformStamped_after = self.tf_buffer.lookup_transform( #先把它轉成odom的座標用的TransformStamped
                            msg.header.frame_id, # target frame Moving frame #也就是現在的'camera_link_optical',    
                            'odom',
                            rclpy.time.Time() , # world frame Original frame (typically fixed)（作為固定的參考, 才知道轉了多少)
                            #rclpy.duration.Duration(seconds=0)  # Correct way to create a Duration object
                            )         # Time when the point cloud was captured
                        # TransformStamped_between = self.tf_buffer.transform(
                        #         TransformStamped_before,
                        #         TransformStamped_after,  # Make sure we use the correct frame ID
                        #         False
                        #     )

                        # source_pointcloud2_transformed_points_np = self.transform_pointcloud_to_np2(source_points_np, TransformStamped)
                        
                        # source_pointcloud2_transformed_points_np1 = self.transform_pointcloud_to_np(source_points_np, TransformStamped_before)
                        # source_pointcloud2_transformed_points_np = self.transform_pointcloud_to_np(source_pointcloud2_transformed_points_np, TransformStamped_after)
                        
                        
                        # source_pointcloud2_transformed_points_np = self.transform_pointcloud_to_np(source_pointcloud2_transformed_points_np1, TransformStamped_after)
                        
                        # source_pointcloud2_transformed_points_np = self.transform_pointcloud_to_np(source_points_np, TransformStamped_between)
                        # try_header=std_msgs.Header(frame_id='odom')
                        # self.get_logger().info("ok1")
                        # source_pointcloud2__transformed_points= pc2.create_cloud(try_header, msg.fields, source_pointcloud2_transformed_points_np.tolist())
                        # self.get_logger().info("ok2")

                        source_pointcloud2_points= pc2.create_cloud(msg.header, msg.fields, source_points_np.tolist())
                        self.tompcd_ICPonly_pub_.publish(source_pointcloud2_points)
                        self.get_logger().info("ok3")
                        
                        msg_pointcloud2 = pc2.create_cloud(msg.header, msg.fields, msg_points_np.tolist())
                        whole_pointcloud2 = pc2.create_cloud(msg.header, msg.fields, whole_points_np.tolist())
                        
                        self.tompcd_ICP_pub_.publish(msg_pointcloud2) #現在這個沒有publish estimated tomato了
                        # self.tompcd_ICP_pub_.publish(whole_pointcloud2)
                        if __debug__:
                            self.get_logger().info("done publishing whole_pointcloud2 to ICP_topic")
                    else: 
                        self.get_logger().info("Too few correspondent point in global registration, fail to ICP")
                        self.get_logger().info("Continue to detect filtered point...")
                # except Exception as e: #如果點太少ICP失敗, 就說ICP失敗然後continue
                #     self.get_logger().info("The error is: %s" % str(e))
                #     self.get_logger().info("fail to ICP")
                #     self.get_logger().info("Continue to detect filtered point...")
                





        #==========================================
        # pc_array = np.array(int_data)
        # num_points_pc = len(pc_array) // 4  # Assuming 4 values per point
        # pc_array = pc_array.reshape((num_points_pc, 4)) #<Debug>現在pc_array是一個list x y z rgb 一直列下去, 但是要成pointcloud2 data要是[x,y,z,rgb]的排排列, 所以需要每4個一組reshap(gen也是一個陣列直直列所以也要reshape)
        # pc_list=pc_array.tolist()
        
        # BIT_MOVE_16 = 2**16
        # BIT_MOVE_8 = 2**8
        # #convert open3d to numpy array
        # points_o3d = np.asarray(source.points)
        # if source.colors:
        #     # colors_o3d = np.asarray(source.colors)
        #     colors_o3d = np.floor(np.asarray(source.colors)*255) # nx3 matrix
        #     colors_o3d = colors_o3d[:,0] * BIT_MOVE_16 +colors_o3d[:,1] * BIT_MOVE_8 + colors_o3d[:,2]  
        #     colors_o3d = colors_o3d.reshape(-1, 1)
        # # else:
        # #     colors_o3d = np.zeros_like(points)
        
        # self.get_logger().info(f'Shape of pc_array: {points_o3d.shape}')
        # self.get_logger().info(f'Shape of new_point: {(colors_o3d.shape)}')
        # # new_point_array=np.hstack((points_o3d, colors_o3d))
        # result_model = np.append(points_o3d, colors_o3d, axis=1)
        # # Example: Assuming result_model has fields 'x', 'y', 'z', 'rgb'
        # # result_numeric = np.array([result_model['x'], result_model['y'], result_model['z'], result_model['rgb']]).T
        # # Ensure result_numeric is of type float64
        # # result_numeric = result_model.astype(np.float64)
        # # Assuming result_numeric has shape (n, 4)
        # # Ensure result_numeric has the correct shape (m, 4)
        # assert result_model.shape[1] == 4, "result_model should have 4 columns for x, y, z, rgb"
        # structured_result = np.zeros(result_model.shape[0], dtype=pc_array.dtype)

        # # structured_result = np.zeros(result_model.shape[0], dtype=pc_array.dtype)
        # structured_result['x'] = result_model[:, 0]
        # structured_result['y'] = result_model[:, 1]
        # structured_result['z'] = result_model[:, 2]
        # structured_result['rgb'] = result_model[:, 3]
        # self.get_logger().info(f'.dtype of combined_numpy: {(result_model.dtype)}')
        # # self.get_logger().info(f'.dtype of combined_numpy: {(result_numeric.dtype)}')
        # self.get_logger().info(f'.dtype of combined_numpy: {(pc_array.dtype)}')
        # self.get_logger().info(f'Shape of pc_array: {pc_array.shape}')
        # self.get_logger().info(f'Shape of new_point: {(result_model.shape)}')
        # self.get_logger().info(f'Shape of structured_result: {(structured_result.shape)}')

        # combined_numpy=np.concatenate(pc_array, structured_result)
        # # self.get_logger().info(f'.dtype of combined_numpy: {(combined_numpy.dtype)}')
        # # #Convert NumPy Array to PointCloud2 Message
        # combined_list = combined_numpy.tolist()
        # Integrated_msg = pc2.create_cloud(msg.header, msg.fields, combined_list)
        # self.tompcd_ICP_pub_.publish(Integrated_msg)
        
        






        #不知了反正new_point是事項上面一樣用append的
        # new_point = list(pc2.read_points(source_pointcloud2, field_names=("x", "y", "z", "rgb"), skip_nans=False))

        # for point in new_point:
        #     x, y, z, rgb = point
        #     the_point=[x, y, z, rgb]
        #     pc_list.append(the_point)




        # #<Debug> 不知道為啥一直說我的array size是3649但明明、應該會是4的倍數因為x,y,z,rgb但是不知為啥, 反正因為這樣不能reshape了, 所以就強至刪掉不整除的部份看看
        # # num_points_new = len(new_point) //4
        # # new_point = new_point[(len(new_point)%4):]
        # # new_point = new_point.reshape((num_points_new, 4))
        # # new_array_1d = array_1d[:-3]
        # # if new_point.ndim == 1:
        # #     new_point = new_point.reshape((-1, 4))
        
        
        # self.get_logger().info(f'PointCloud2 fields: {msg.fields}')
        # self.get_logger().info(f'Shape of pc_array: {pc_array.shape}')
        # # self.get_logger().info(f'Shape of new_point: {(new_point.shape)}')

        # # pc_array = np.vstack((pc_array, new_point))
        # Integrated_msg = pc2.create_cloud(msg.header, msg.fields, pc_list)
        # self.tompcd_ICP_pub_.publish(Integrated_msg)
        
        
        #o3d.io.write_point_cloud("/home/jajayu/Documents/NextBestView/pointCloudDataPLY/copy_of_filtered_msg2.pcd",point_cloud)
        #==============================================




    def transform_pointcloud_to_np2(self, source_points_np, transform_stamped):
        """
        Transform a point cloud (represented as a numpy array) from the source frame to the target frame
        using the given transformation (TransformStamped).
        
        :param source_points_np: numpy.ndarray
            A numpy array of shape (N, 3) where each row is a point (x, y, z) in the source frame.
            
        :param transform_stamped: TransformStamped
            The transformation to be applied (from tf2).
        
        :return: numpy.ndarray
            A numpy array of shape (N, 3) with the transformed points in the target frame.
        """
        
        # Create an empty list to store the transformed points
        transformed_points = []
        

        for point in source_points_np:
            # Convert numpy array point to a Point message
            x, y, z, rgb = point
            point_wrt_source = Point(x=x, y=y, z=z)
            # point_wrt_source = Point(x=point[0], y=point[1], z=point[2], rgb=point[3])
            
            # Transform the point using tf2
            point_wrt_target = tf2_geometry_msgs.do_transform_point(
                PointStamped(point=point_wrt_source), transform_stamped).point
            
            # Store the transformed point in the list
            transformed_points.append([point_wrt_target.x, point_wrt_target.y, point_wrt_target.z, rgb])
            # transformed_points.append()
        # Convert the transformed points list back to a numpy array
        transformed_points_np = np.array(transformed_points)
        
        return transformed_points_np

    # def transform_pointcloud_to_np(self,source_points_np, transform_stamped):
    def transform_pointcloud_to_np(self,source_points_np, transform_stamped):
        """source_pointcloud2
        Transforms a PointCloud2 message to a new frame using a TransformStamped object
        and returns the transformed points as a NumPy array.

        :param pointcloud: The input PointCloud2 message to transform.
        :param transform_stamped: The TransformStamped containing the transformation.
        :return: A NumPy array of transformed points with shape (N, 3).
        """
        
        # Extract translation and rotation from TransformStamped
        trans = transform_stamped.transform.translation
        rot = transform_stamped.transform.rotation
        translation = np.array([trans.x, trans.y, trans.z])
        # quaternion = np.array([rot.x, rot.y, rot.z, rot.w])
        #Convert quaternion to a 3x3 rotation matrix
        rotation = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()

        # Create a 4x4 transformation matrix
        # transformation_matrix = tf_transformations.quaternion_matrix(quaternion)
        # transformation_matrix[0:3, 3] = translation
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation
        transformation_matrix[:3, 3] = translation

        x = source_points_np['x']
        y = source_points_np['y']
        z = source_points_np['z']
        rgb = source_points_np['rgb']
        #重新stack成一維陣列然後把它轉置（變成每個point是)直的x y z
        # Stack x, y, z components into a 2D numpy array
        points_np_xyz = np.vstack((x, y, z)).T
        #這樣是成 
        # [[x, y, z],
        #  [x, y, z],
        #         ...]
        # Transform each point
        num_points = points_np_xyz.shape[0] #source_points_np.shape[0]
        points_homogeneous_xyz = np.hstack((points_np_xyz, np.ones((num_points, 1))))#為了拿去乘transformation matrix, 所以要疊成直的x,y,z,1
        transformed_points_homogeneous_xyz = (transformation_matrix @ points_homogeneous_xyz.T).T #transformation_matrix.T #不確定這邊幹麻再轉置, 我覺得不用
        transformed_points = transformed_points_homogeneous_xyz[:, :3] #再把xyz轉置回來後只取前面xyz的部份(現在是一點橫橫的陣列去疊)

        structured_dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)])

        # Create an empty structured array with shape (200, 1) and the defined dtype
        structured_array = np.zeros(num_points, dtype=structured_dtype) #因為橫的有xyz項所以變成陣列後要＊3
        # flattened_points = transformed_points.flatten()
        structured_array['x'] = transformed_points[:, 0]#.reshape(-1, 1) [Debug] .reshape(-1, 1)會把東東變成[2360,1]（2D array) 不等於[2360,] (1D array!！!)
        structured_array['y'] = transformed_points[:, 1]#.reshape(-1, 1)  
        structured_array['z'] = transformed_points[:, 2]#.reshape(-1, 1)
        structured_array['rgb'] = rgb#.reshape(-1, 1)
        print(structured_array)

        return structured_array#transformed_points
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


    def convert_open3d_to_ros_pointcloud2(self, open3d_pcd):
        # Extract point cloud data
        points = np.asarray(open3d_pcd.points)
        colors = np.asarray(open3d_pcd.colors)

        header=std_msgs.Header(frame_id='camera_link_optical')#, stamp=Clock().now().to_msg())#cam_frame_pcd(for realsense）#設定這個點雲圖會顯示到的frame ID      
        
        # Convert to ROS2 PointCloud2
        # # header = rclpy.header.Header(frame_id='your_frame_id')
        # fields = [
        #     pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
        #     pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
        #     pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
        #     pc2.PointField(name='r', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
        #     pc2.PointField(name='g', offset=16, datatype=pc2.PointField.FLOAT32, count=1),
        #     pc2.PointField(name='b', offset=20, datatype=pc2.PointField.FLOAT32, count=1),
        # ]
        fields = [
        # sensor_msgs.PointField(
        # name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        # for i, n in enumerate('xyz'),
        sensor_msgs.PointField(name='x', offset=0, datatype=np.dtype, count=1),
        sensor_msgs.PointField(name='y', offset=4, datatype=np.dtype, count=1),
        sensor_msgs.PointField(name='z', offset=8, datatype=np.dtype, count=1),
        sensor_msgs.PointField(name='rgb', offset=12, datatype=sensor_msgs.PointField.UINT32, count=1)
        ]

        # Merge points and colors into a single array
        point_cloud_data = np.hstack([points, colors])

        # Create PointCloud2 message
        point_cloud2_msg = pc2.create_cloud(header, fields, point_cloud_data)
        # filtered_msg = pc2.create_cloud(msg.header, msg.fields, filtered_points)
        return point_cloud2_msg


# def pcd_to_pointcloud2(points, colors, parent_frame):
#     ros_dtype = sensor_msgs.PointField.FLOAT32
#     dtype = np.float32
#     itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.
#     #data = points.astype(dtype).tobytes() 
#     #The fields specify what the bytes represents. The first 4 bytes 
#     # represents the x-coordinate, the next 4 the y-coordinate, etc.
#     fields = [
#         # sensor_msgs.PointField(
#         # name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
#         # for i, n in enumerate('xyz'),
#         sensor_msgs.PointField(name='x', offset=0, datatype=ros_dtype, count=1),
#         sensor_msgs.PointField(name='y', offset=4, datatype=ros_dtype, count=1),
#         sensor_msgs.PointField(name='z', offset=8, datatype=ros_dtype, count=1),
#         sensor_msgs.PointField(name='rgb', offset=12, datatype=sensor_msgs.PointField.UINT32, count=1)
#     ]

#     point_data=[]
#     #PointCloud2 color data eat HSV, so need to convert to HSV first
#     for i in range(len(points)):
#         x,y,z=points[i]
#         r,g,b=colors[i]*255
#         #print("r:"+(string)r+"g:"+g+"b:"+b)
#         # convert to HSV
#         # r,g,b=colorsys.rgb_to_hsv(r, g, b)
        
#         # r=0
#         # g=0
#         # b=255
#         rgb=(int(r)<<16 | (int(g)<<8 | int(b))) # / /r
#         # a=100
#         # rgb = struct.unpack('I', struct.pack('BBBB', a,r,g,b))[0]
#         point_data.append([x, y, z, rgb])
#         #g/ /r/
#     #print(point_data)


#     # The PointCloud2 message also has a header which specifies which 
#     # coordinate frame it is represented in. 
#     header = std_msgs.Header(frame_id=parent_frame)

#     msg = sensor_msgs.PointCloud2(
#         header=header,
#         height=1, 
#         width=points.shape[0],
#         is_dense=True,
#         is_bigendian=False,
#         fields=fields,
#         point_step=16, #(itemsize * 4), # Every point consists of three float32s.
#         row_step=(itemsize * 4 * points.shape[0]),
#         data=np.asarray(point_data, dtype=np.float32).tobytes()
#     )


#     return msg

def main(args=None): #construct main function
    # print("everything alright")
    
    # source_path="/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/ros2_ws2/src/dataset/data_pcd/TomatoPlant_size_modified_only1tomato.ply"
    # target_path="/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/ros2_ws2/src/dataset/data_pcd/copy_of_filtered_msg.pcd"
    # num=3

    # # demo_pcds = o3d.data.DemoICPPointClouds()
    # # source = o3d.io.read_point_cloud(demo_pcds.paths[0])
    # # target = o3d.io.read_point_cloud(demo_pcds.paths[1])

    # source = o3d.io.read_point_cloud(source_path.format(num)) #demo_pcds.paths[0])
    # target = o3d.io.read_point_cloud(target_path.format(num)) #demo_pcds.paths[1])

    # ICPoperation(source,target)
    # =============== [下面這段是run in optimize mode] ================
    if not __debug__:
        # Your main code here
        print("Running in optimized mode")
        # Proceed with normal node logic
    else:
        # Relaunch the script in optimized mode
        script_path = os.path.realpath(__file__)
        os.execv(sys.executable, [sys.executable, '-O', script_path] + sys.argv[1:])
    #===============================================================
    
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