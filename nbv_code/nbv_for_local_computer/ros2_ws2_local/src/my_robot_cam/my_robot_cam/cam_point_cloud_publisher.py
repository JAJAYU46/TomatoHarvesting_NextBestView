import open3d as o3d
import numpy as np
import cv2

#For ROS2
#!/user/bin/env python3 
import colorsys
import struct
import rclpy #library for ROS2
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs  #For publisher

import sys
import os

#path_in_pcd = "/home/rmml01/d435_pointcloud/pcd_saved_240523/pcd_{}.ply"
#path_in_pcd = "/home/jajayu/Documents/NextBestView/pointCloudDataPLY/xyzrgb_dragon.ply"
path_in_pcd = "/home/jajayu/Documents/NextBestView/pointCloudDataPLY/copy_of_filtered_msg2.pcd"
#path_in_pcd = "/home/jajayu/Documents/NextBestView/pointCloudDataPLY/Merged mesh.ply"
#path_in_pcd = "/home/jajayu/Documents/NextBestView/pointCloudDataPLY/MinePLY/out.ply"
#path_in_pcd = "/home/jajayu/Documents/NextBestView/pointCloudDataPLY/fragment.ply"
#path_in_pcd = "/home/jajayu/Documents/NextBestView/pointCloudDataPLY/Meshtopoint.ply"
num = 3





#create a node for publishing--node: cam_point_cloud_publisher
class cam_point_cloud_publisher(Node): #construct Node class
    


    def __init__(self): #construct constructor
        super().__init__("cam_point_cloud_publisher") #set python_NodeName
        

        pcd = o3d.io.read_point_cloud(path_in_pcd.format(num))
        self.points = np.asarray(pcd.points)
        self.colors = np.asarray(pcd.colors)
        #self.colors = np.asarray(np.zeros(10))
        print(self.points.shape)
        print(self.colors)
        o3d.visualization.draw_geometries([pcd])

        #【publisher】
        self.pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2, "pcd", 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        timer_period = 5
        #creat callback & its timer
        self.get_logger().info("python_NodeName have been started")
        self.create_timer(timer_period, self.callback_forPublish) #(time interval/ calling callback)
        #self.callback_forPublish()




    # def callback_forSubscribe(self, msg: Pose): #pose_callback #sent in a msg with message_type Pose的msg (msg as a Pose object)
    #     #============【Subscribe】============
    #     self.get_logger().info(str(msg.x))
    #     self.get_logger().info("("+str(msg.x)+", "+str(msg.y)+")")

    def callback_forPublish(self): #construct a callback
        #============【Publish】===========
        msg = sensor_msgs.PointCloud2() #create a message object from the class twist
        
        #every time interval, do the code below to transfer pcd to point cloud
        self.data_pointCloud2=pcd_to_pointcloud2(self.points, self.colors, 'map') ##convertedPCD: variable contains the point cloud data
        #self.get_logger().info("LaLaLa")
        #self.get_logger().info(str(self.data_pointCloud2))
        self.pcd_publisher.publish(self.data_pointCloud2) #The default (fixed) frame in RViz is called 'map'
        #self.cmd_vel_pub_.publish(msg) #publish thing through this publisher topic


def pcd_to_pointcloud2(points, colors, parent_frame):



    # ros_dtype = sensor_msgs.PointField.FLOAT32
    # dtype = np.float32
    # itemsize = np.dtype(dtype).itemsize


    # msg=sensor_msgs.PointCloud2()

    # #Set the message data
    # msg.header.stamp = rclpy.time.Time().to_msg()
    # msg.header.frame_id = 'map'
    # msg.height=1
    # msg.width=points.shape[0]
    # msg.is_dense=False

    # fields = [
    #     # sensor_msgs.PointField(
    #     # name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
    #     # for i, n in enumerate('xyz'),
    #     sensor_msgs.PointField(name='x', offset=0, datatype=ros_dtype, count=1),
    #     sensor_msgs.PointField(name='y', offset=4, datatype=ros_dtype, count=1),
    #     sensor_msgs.PointField(name='z', offset=8, datatype=ros_dtype, count=1),
    #     sensor_msgs.PointField(name='rgb', offset=12, datatype=ros_dtype, count=1)
    # ]
    # msg.fields = fields

    # point_data=[]
    # for i in range(len(points)):
    #     x,y,z=points[i]
    #     r,g,b=colors[i]
    #     rgb=(int(r)<<16 | (int(g)<<8 | int(b)))
    #     point_data.append([x, y, z, rgb])
    # msg.data = np.asarray(point_data, dtype=np.float32).tobytes()
    # msg.point_step=16     #(int)(itemsize * 3), # Every point consists of three float32s.
    # msg.row_step=(int)(itemsize * 3 * points.shape[0])

    # return msg
    













    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.
    #data = points.astype(dtype).tobytes() 
    #The fields specify what the bytes represents. The first 4 bytes 
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [
        # sensor_msgs.PointField(
        # name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        # for i, n in enumerate('xyz'),
        sensor_msgs.PointField(name='x', offset=0, datatype=ros_dtype, count=1),
        sensor_msgs.PointField(name='y', offset=4, datatype=ros_dtype, count=1),
        sensor_msgs.PointField(name='z', offset=8, datatype=ros_dtype, count=1),
        sensor_msgs.PointField(name='rgb', offset=12, datatype=sensor_msgs.PointField.UINT32, count=1)
    ]

    point_data=[]


    # print(colors.dtype)
    # colors = (colors * 255).astype(np.uint8)  # Convert to 0-255 range
    # # Convert RGB to BGR
    # print(colors.shape)
    # colors = cv2.cvtColor(colors.reshape(-1, 1, 3), cv2.COLOR_RGB2BGR).reshape(-1, 3)

    # # Convert BGR to HSV
    # colors_hsv = cv2.cvtColor(colors.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV)

    #PointCloud2 color data eat HSV, so need to convert to HSV first
    for i in range(len(points)):
        x,y,z=points[i]
        r,g,b=colors[i]*255
        #print("r:"+(string)r+"g:"+g+"b:"+b)
        # convert to HSV
        # r,g,b=colorsys.rgb_to_hsv(r, g, b)
        
        # r=0
        # g=0
        # b=255
        rgb=(int(r)<<16 | (int(g)<<8 | int(b))) # / /r
        # a=100
        # rgb = struct.unpack('I', struct.pack('BBBB', a,r,g,b))[0]
        point_data.append([x, y, z, rgb])
        #g/ /r/
    #print(point_data)


    # The PointCloud2 message also has a header which specifies which 
    # coordinate frame it is represented in. 
    header = std_msgs.Header(frame_id=parent_frame)

    msg = sensor_msgs.PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=16, #(itemsize * 4), # Every point consists of three float32s.
        row_step=(itemsize * 4 * points.shape[0]),
        data=np.asarray(point_data, dtype=np.float32).tobytes()
    )


    return msg


def filter_point_cloud_by_hsv(points, colors, min_hsv, max_hsv):
    """
    Remove points in the point cloud within the specified HSV color range.

    Args:
    - points (np.ndarray): The input point cloud points.
    - colors (np.ndarray): The colors associated with the points.
    - min_hsv (np.array): The minimum HSV color range.
    - max_hsv (np.array): The maximum HSV color range.

    Returns:
    - np.ndarray: The filtered points.
    - np.ndarray: The filtered colors.
    """
    print(colors.dtype)
    colors = (colors * 255).astype(np.uint8)  # Convert to 0-255 range
    # Convert RGB to BGR
    print(colors.shape)
    colors = cv2.cvtColor(colors.reshape(-1, 1, 3), cv2.COLOR_RGB2BGR).reshape(-1, 3)

    # Convert BGR to HSV
    colors_hsv = cv2.cvtColor(colors.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV)

    # Create mask for colors within the HSV range
    mask = cv2.inRange(colors_hsv, min_hsv, max_hsv)

    # Select points not in the color range
    filtered_points = []
    filtered_colors = []
    for i in range(len(mask)):
        if mask[i] != 0:
            filtered_points.append(points[i])
            filtered_colors.append(colors[i])

    filtered_points = np.array(filtered_points)
    filtered_colors = np.array(filtered_colors) / 255

    return filtered_points, filtered_colors



def pcd_to_pointcloud():
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(path_in_pcd.format(num))
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    print(f"\nOriginal cloud size: {len(points)} points")

    # Define HSV color range to filter out (e.g., remove all points with a specific hue range)
    # min_hsv = np.array([0, 87, 65])  # Minimum HSV color to filter
    # min_hsv = np.array([70, 166, 88])  # Minimum HSV color to filter

    # min_hsv = np.array([70, 106, 131])  # Minimum HSV color to filter
    # max_hsv = np.array([179, 255, 255])  # Maximum HSV color to filter
    min_hsv = np.array([100, 0,100])  # Minimum HSV color to filter
    max_hsv = np.array([255, 200, 255])  # Maximum HSV color to filter
    # Filter the point cloud by HSV color
    filtered_points, filtered_colors = filter_point_cloud_by_hsv(points, colors, min_hsv, max_hsv)
    print(f"Filtered cloud size: {len(filtered_points)} points")
    print(f"\nfiltered_points = {filtered_points}")

    # Create a new point cloud with filtered points
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
    filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)

    # Save the filtered point cloud
    #o3d.io.write_point_cloud(path_out_pcd.format(num), filtered_pcd)

    # Visualize the filtered point cloud
    o3d.visualization.draw_geometries([pcd])
    o3d.visualization.draw_geometries([filtered_pcd])
    return filtered_pcd

# def main():
#     # Load the point cloud
#     pcd = o3d.io.read_point_cloud(path_in_pcd.format(num))
#     points = np.asarray(pcd.points)
#     colors = np.asarray(pcd.colors)
#     print(f"\nOriginal cloud size: {len(points)} points")

#     # Define HSV color range to filter out (e.g., remove all points with a specific hue range)
#     # min_hsv = np.array([0, 87, 65])  # Minimum HSV color to filter
#     # min_hsv = np.array([70, 166, 88])  # Minimum HSV color to filter

#     # min_hsv = np.array([70, 106, 131])  # Minimum HSV color to filter
#     # max_hsv = np.array([179, 255, 255])  # Maximum HSV color to filter
#     min_hsv = np.array([100, 0,100])  # Minimum HSV color to filter
#     max_hsv = np.array([255, 200, 255])  # Maximum HSV color to filter
#     # Filter the point cloud by HSV color
#     filtered_points, filtered_colors = filter_point_cloud_by_hsv(points, colors, min_hsv, max_hsv)
#     print(f"Filtered cloud size: {len(filtered_points)} points")
#     print(f"\nfiltered_points = {filtered_points}")

#     # Create a new point cloud with filtered points
#     filtered_pcd = o3d.geometry.PointCloud()
#     filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
#     filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)

#     # Save the filtered point cloud
#     #o3d.io.write_point_cloud(path_out_pcd.format(num), filtered_pcd)

#     # Visualize the filtered point cloud
#     o3d.visualization.draw_geometries([pcd])
#     o3d.visualization.draw_geometries([filtered_pcd])

def main(args=None): #construct main function
	rclpy.init(args=args)
	node1 = cam_point_cloud_publisher() #node1=NodeClass: MyNode
	rclpy.spin(node1) #keep node alive until ctrl+C
	rclpy.shutdown()


if __name__ == "__main__":
    main()
