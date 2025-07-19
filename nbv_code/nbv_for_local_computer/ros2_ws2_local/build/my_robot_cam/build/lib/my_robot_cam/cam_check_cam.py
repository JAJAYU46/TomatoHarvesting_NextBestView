import pyrealsense2 as rs
import numpy as np
import cv2

import rclpy #library for ROS2
from rclpy.node import Node

# class MyNode(Node): #construct Node class
# 	def __init__(self): #construct constructor
# 		super().__init__("python_NodeName") #set python_NodeName
		
# 		#【publisher】
# 		self.cmd_vel_pub_=self.create_publisher(Twist, "/turtle/cmd_vel", 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
# 		#【subscriber】
# 		self.pose_subscriber_=self.create_subscription(Pose, "/turtle/cmd_vel", callback_forSubscribe, 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
		
# 		self.get_logger().info("python_NodeName have been started")
# 		self.create_timer(1.0, self.callback1) #(time interval/ calling callback)
		
		
# 	def callback1(self): #construct a callback
# 		#============【Publish】===========
# 		msg = Twist() #create a message object from the class twist
# 		msg.linear.x = 2.0 #fill data of the message
# 		msg.angular.z = 1.0
# 		self.cmd_vel_pub_.publish(msg) #publish thing through this publisher topic
			
# 	def callback_forSubscribe(self, msg: Pose): #pose_callback #sent in a msg with message_type Pose的msg (msg as a Pose object)
# 		#============【Subscribe】============
# 		self.get_logger().info(str(msg.x))
# 		self.get_logger().info("("+str(msg.x)+", "+str(msg.y)+")")
		
		
# def main(args=None): #construct main function
# 	rclpy.init(args=args)
# 	node1 = MyNode() #node1=NodeClass: MyNode
# 	rclpy.spin(node1) #keep node alive until ctrl+C
# 	rclpy.shutdown()

# if __name__=='__main__':
#         main()	
		


pipe = rs.pipeline()
cfg  = rs.config()

cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640,480, rs.format.z16, 30)

pipe.start(cfg)

while True:
    
    frame = pipe.wait_for_frames(timeout_ms=50000)
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,
                                     alpha = 0.5), cv2.COLORMAP_JET)

    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    cv2.imshow('rgb', color_image)
    cv2.imshow('depth', depth_cm)

    if cv2.waitKey(1) == ord('q'):
        break

pipe.stop()