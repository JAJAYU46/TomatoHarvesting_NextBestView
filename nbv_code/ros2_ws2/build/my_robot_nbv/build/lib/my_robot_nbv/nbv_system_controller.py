#This is the controller that will publish the moving status and all node status on "/nbv/status_communicator" topic
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
        
        self.get_logger().info("node 'nbv_tompcd_filter' have been started")
        self.status_subscription = self.create_subscription(NodeStatus, '/nbv/status_communicator',self.status_callback,10)
        # self.bboxReadyFlag = False
        # Start a separate thread to read user input
        # input_thread = threading.Thread(target=self.read_user_input, daemon=True)
        # input_thread.start()

    def read_user_input(self):
        while True:
            user_input = input("Enter 'm' to publish True, 's' to publish False: ").strip()
            msg_status = NodeStatus()
            if user_input == 'm':
                
                msg_status.is_moving = True
                # msg_box.lu_y = TargetBox[1]
                # msg_box.rd_x = TargetBox[2]
                # msg_box.rd_y = TargetBox[3]
                
                self.publish_status_.publish(msg_status)
            elif user_input == 's':
                msg_status.is_moving = False
                self.publish_status_.publish(msg_status)
            else:
                self.get_logger().warn("Invalid input. Please enter 'm' or 's'.")


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


