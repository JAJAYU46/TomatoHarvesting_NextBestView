#!/user/bin/env python3 
import rclpy #library for ROS2
from rclpy.node import Node
import sys
import os

sys.path.append(os.path.dirname(os.path.realpath(__file__))) #approach adds the current directory where the script is located to the Python path.

from message_interfaces.msg import NodeStatus    # CHANGE
import threading


class MyNode(Node): #construct Node class
    def __init__(self): #construct constructor
        super().__init__("nbv_tompcd_filter") #set python_NodeName
        
        
        #[[For ROS2 publisher and subscriber]]
        #【publisher】
        self.publish_status_=self.create_publisher(NodeStatus, "/nbv/status_communicator", 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        self.status_subscription = self.create_subscription(NodeStatus, '/nbv/status_communicator',self.status_callback,10)
        
        self.get_logger().info("node 'nbv_tompcd_filter' have been started")
        # self.bbox_subscription = self.create_subscription(BoundingBox, '/nbv/bounding_box_msg',self.bbox_callback,10)
        # self.bboxReadyFlag = False
        # Start a separate thread to read user input
        input_thread = threading.Thread(target=self.read_user_input, daemon=True)
        input_thread.start()

    def read_user_input(self):
        while True:
            user_input = input("Enter 'm' to publish True, 's' to publish False: ").strip()
            msg_status = NodeStatus()
            while(self.renew_done != True): #要等確保是最新資料
                    continue
            if user_input == 'm':
                
                # msg_status.is_moving = True
                # msg_box.lu_y = TargetBox[1]
                # msg_box.rd_x = TargetBox[2]
                # msg_box.rd_y = TargetBox[3]
                # msg_status = NodeStatus()
                
                msg_status.ready_for_next_iteration = self.ready_for_next_iteration_msg #(因為還是要繼續偵測, 所以留著)
                msg_status.is_moving = True #只改這個
                msg_status.iteration = self.iteration_msg
                msg_status.detection_done = self.detection_done_msg
                msg_status.icp_done = self.icp_done_msg  #只改這個
                msg_status.octomap_done = self.octomap_done_msg# 因為這個包是直接用octomap server2的, 所以只有它是在之後nbv的時候會被改著定義
                msg_status.nbv_done = self.nbv_done_msg
                msg_status.nbv_point_x = self.nbv_point_x_msg #這當預設的反正這個到時候是base也不可能
                msg_status.nbv_point_y = self.nbv_point_y_msg
                msg_status.nbv_point_z = self.nbv_point_z_msg
                msg_status.is_final_result = self.is_final_result_msg
                self.publish_status_.publish(msg_status)

            elif user_input == 's':
                msg_status.is_moving = False
                self.publish_status_.publish(msg_status)
            else:
                self.get_logger().warn("Invalid input. Please enter 'm' or 's'.")
            self.renew_done=False
    def status_callback(self,msg):
        if (self.is_moving_msg != msg.is_moving): 
            self.get_logger().info('now the is_moving_msg: '+str(self.is_moving_msg)) # CHANGE
        self.ready_for_next_iteration_msg = msg.ready_for_next_iteration
        self.is_moving_msg = msg.is_moving
        self.iteration_msg = msg.iteration
        self.detection_done_msg = msg.detection_done
        self.icp_done_msg = msg.icp_done
        self.octomap_done_msg = msg.octomap_done # 因為這個包是直接用octomap server2的, 所以只有它是在之後nbv的時候會被改著定義
        self.nbv_done_msg = msg.nbv_done
        self.nbv_point_x_msg = msg.nbv_point_x
        self.nbv_point_y_msg = msg.nbv_point_y
        self.nbv_point_z_msg = msg.nbv_point_z
        self.is_final_result_msg = msg.is_final_result
        
        self.renew_done = True
    

def main(args=None): #construct main funct沒有動ㄝ
    
    # =============== [下面這段是run in optimize mode] ================
    # if not __debug__:
    #     # Your main code here
    #     print("Running in optimized mode")
    #     # Proceed with normal node logic
    # else:
    #     # Relaunch the script in optimized mode
    #     script_path = os.path.realpath(__file__)
    #     os.execv(sys.executable, [sys.executable, '-O', script_path] + sys.argv[1:])
    #===============================================================
    
    rclpy.init(args=args)
    node1 = MyNode() #node1=NodeClass: MyNode
    rclpy.spin(node1) #keep node alive until ctrl+C
    rclpy.shutdown()
    

if __name__=='__main__':
        main()	


