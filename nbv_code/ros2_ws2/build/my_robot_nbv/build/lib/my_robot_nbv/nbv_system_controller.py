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
        
        # [Status Variable]
        self.ready_for_next_iteration_msg = True
        self.is_moving_msg=False  # <Debug> msg field 要is_moving才行, 不可isMoving
        self.iteration_msg=1000 #這個先這樣保險一點
        # self.detection_done_msg
        # self.icp_done_msg
        # self.octomap_done_msg# 因為這個包是直接用octomap server2的, 所以只有它是在之後nbv的時候會被改著定義
        # self.nbv_done_msg
        # self.nbv_point_x_msg
        # self.nbv_point_y_msg
        # self.nbv_point_z_msg
        # self.is_final_result_msg

        self.doneReset=False
        self.ready_for_new_Tomato = True
        # self.status_reset()
        
        
        # self.bboxReadyFlag = False
        # Start a separate thread to read user input
        interface_thread = threading.Thread(target=self.interface_info, daemon=True)
        interface_thread.start()
    
    



    def status_callback(self,msg):
        self.ready_for_next_iteration_msg = msg.ready_for_next_iteration
        if (self.is_moving_msg != msg.is_moving): 
            self.get_logger().info('now the is_moving_msg: '+str(self.is_moving_msg)) # CHANGE
          
        self.is_moving_msg = msg.is_moving
        self.iteration_msg = msg.iteration
        self.detection_done_msg = msg.detection_done
        self.icp_done_msg = msg.icp_done
        self.octomap_done_msg = msg.octomap_done# 因為這個包是直接用octomap server2的, 所以只有它是在之後nbv的時候會被改著定義
        self.nbv_done_msg = msg.nbv_done
        self.nbv_point_x_msg = msg.nbv_point_x
        self.nbv_point_y_msg = msg.nbv_point_y
        self.nbv_point_z_msg = msg.nbv_point_z
        self.is_final_result_msg = msg.is_final_result

        self.doneReset = True

    def status_reset(self): #初始化所有參數, 再把iteration設為0（表示前一個iteration已完成, 可執行下移步驟後, 各個node才會接續著跟著動作）
          
        msg_status = NodeStatus()
        msg_status.ready_for_next_iteration = True
        msg_status.is_moving = False
        msg_status.iteration = 0
        msg_status.detection_done = False
        msg_status.icp_done = False
        msg_status.octomap_done = False# 因為這個包是直接用octomap server2的, 所以只有它是在之後nbv的時候會被改著定義
        msg_status.nbv_done = False
        msg_status.nbv_point_x = 0.0 #這當預設的反正這個到時候是base也不可能
        msg_status.nbv_point_y = 0.0
        msg_status.nbv_point_z = 0.0
        msg_status.is_final_result = False
        self.publish_status_.publish(msg_status)
        self.get_logger().info('Reset Status1 done') # CHANGE
        
        # self.doneReset = True
    def status_reset_within_iteration(self): #初始化所有參數, 再把iteration設為0（表示前一個iteration已完成, 可執行下移步驟後, 各個node才會接續著跟著動作）
        msg_status = NodeStatus()
        msg_status.ready_for_next_iteration = True
        msg_status.is_moving = self.is_moving_msg
        msg_status.iteration = self.iteration_msg+1
        msg_status.detection_done = False
        msg_status.icp_done = False
        msg_status.octomap_done = False# 因為這個包是直接用octomap server2的, 所以只有它是在之後nbv的時候會被改著定義
        msg_status.nbv_done = False
        msg_status.nbv_point_x = self.nbv_point_x_msg #這當預設的反正這個到時候是base也不可能
        msg_status.nbv_point_y = self.nbv_point_y_msg
        msg_status.nbv_point_z = self.nbv_point_z_msg
        msg_status.is_final_result = self.is_final_result_msg
        self.publish_status_.publish(msg_status)
        self.get_logger().info('Reset Status2 done') # CHANGE
        
        # self.doneReset = True

    def interface_info(self):
        
        while True:
            # 先初始化, 讓整個行動開始
            if(self.ready_for_new_Tomato==True):
                user_input = input("Enter 'n' to start NBV process for next target tomato: ").strip()
                self.status_reset()
                self.ready_for_new_Tomato=False
            else: 
                self.status_reset_within_iteration()
            


            #1. 要iteration_msg==0
            while(self.doneReset!=True): #會一直等直到被初始化完成
                continue
            while(self.ready_for_next_iteration_msg!=True): #會一直等直到被初始化完成
                continue
            
            self.get_logger().info("Done initializing this scene")
            self.get_logger().info("start for the "+str(self.iteration_msg)+"th iteration")
            
            

            #2. 等待完成detection =========================================
            self.get_logger().info("Doing the detection for tomato...")
            while(self.detection_done_msg != True): #等待完成
                continue
            self.get_logger().info("Done Detection")

            
            
            #3. 等待完成filter & ICP  ======================================
            self.get_logger().info("Doing the filter & ICP for tomato...")
            while(self.icp_done_msg != True): #等待完成ICP
                continue  
            self.get_logger().info("Done Detection")

            #4. 等待完成Octomap  ======================================
            self.get_logger().info("Doing the pcd to Octomap convertion...")
            while(self.octomap_done_msg != True): #等待完成ICP
                continue  
            self.get_logger().info("Done Octomap")

            #5. 等待完成NBV calculation  ======================================
            self.get_logger().info("Calculating NBV toward target tomato...")
            while(self.nbv_done_msg != True): #等待完成ICP
                continue  
            self.get_logger().info("Done Calculating NBV toward target tomato")
            self.get_logger().info("============== Result =============")
            self.get_logger().info("Iteration: "+str(self.iteration_msg)+"th")
            # self.get_logger().info("The NBV for the tomato is: ( %.2f, %.2f, %.2f)",(self.nbv_point_x_msg),(self.nbv_point_y_msg), (self.nbv_point_z_msg))
            self.get_logger().info(f"The NBV for the tomato is: ( {self.nbv_point_x_msg:.2f}, {self.nbv_point_y_msg:.2f}, {self.nbv_point_z_msg:.2f} )")
            self.get_logger().info("===================================")

            if(self.is_final_result_msg==False): 
                self.get_logger().info("SENDINT COMMAND TO ROBOT ARM...")
                while(self.is_moving_msg != True): #等待變成true表示在move
                    continue  
                self.get_logger().info("MOVING THE ROBOT ARM...")
                while(self.is_moving_msg != False): #等待移動好變回靜止
                    continue  
                self.get_logger().info("CAMERA ARRIVE BEST VIEW POSITION")
                
                self.doneReset=False
                # self.status_reset_within_iteration() #didn't reset iteration... x, y, z
                self.get_logger().info("Done Reset starting next iteration")
                # ready for...
                
            else: 
                self.get_logger().info("Arrive the final position for grabbing")
                self.get_logger().info(f"The NBV for the tomato is: ( {self.nbv_point_x_msg:.2f}, {self.nbv_point_y_msg:.2f}, {self.nbv_point_z_msg:.2f} )")
                # self.status_reset() #不可以在這裡reset, 要等下令
                # self.get_logger().info("Done Reset, starting NBV for next Target Tomato scene")
                self.ready_for_new_Tomato = True
                # user_input = input("Enter 'n' to start NBV process for next target tomato: ").strip()
                
                # ready for...


            # 看這是不是最終位置了 要再做一次NBV才會知道上一次已經是最終位置了, 
            # 所以當is_final_result_msg是true時, 就不用移動robot arm了（因為在上一次就到達這裡了）
            if(self.is_final_result_msg==True): 
                self.get_logger().info("")









            
            # user_input = input("Enter 'm' to publish True, 's' to publish False: ").strip()
            # msg_status = NodeStatus()
            # if user_input == 'm':
                
            #     msg_status.is_moving = True
            #     # msg_box.lu_y = TargetBox[1]
            #     # msg_box.rd_x = TargetBox[2]
            #     # msg_box.rd_y = TargetBox[3]
                
            #     self.publish_status_.publish(msg_status)
            # elif user_input == 's':
            #     msg_status.is_moving = False
            #     self.publish_status_.publish(msg_status)
            # else:
            #     self.get_logger().warn("Invalid input. Please enter 'm' or 's'.")


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


