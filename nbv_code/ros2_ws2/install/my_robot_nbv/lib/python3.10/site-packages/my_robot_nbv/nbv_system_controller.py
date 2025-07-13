#This is the controller that will publish the moving status and all node status on "/nbv/status_communicator" topic
#!/user/bin/env python3 
import rclpy #library for ROS2
from rclpy.node import Node
import sys
import os

sys.path.append(os.path.dirname(os.path.realpath(__file__))) #approach adds the current directory where the script is located to the Python path.

from message_interfaces.msg import NodeStatus    # CHANGE
import threading

# 計實用
import time
import numpy as np
# 會顯示的時間
'''
分次報表
1. 第幾iterate, 分別(detect, filter, octomap, nbv 花的時間), 這個iteration花的總時間, 得到的 temp nbv point
最終結果報表
1. 花的iterate數
2. 花的總時間
3. 最終的nbv point

'''


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

        # [time controller]
        self.time_start = None
        self.time_itter_start = None
        self.time_temp_section_start = None

        self.time_results = []
        self.nbv_results = []
        
        
        # self.bboxReadyFlag = False
        # Start a separate thread to read user input
        interface_thread = threading.Thread(target=self.interface_info, daemon=True)
        interface_thread.start()
    
    



    def status_callback(self,msg):
        self.ready_for_next_iteration_msg = msg.ready_for_next_iteration
        if (self.is_moving_msg != msg.is_moving): 
            self.get_logger().info('now the is_moving_msg: '+str(msg.is_moving)) # CHANGE
        
        self.is_moving_msg = msg.is_moving
        self.target_box_id_msg = msg.target_box_id
        self.iteration_msg = msg.iteration
        self.detection_done_msg = msg.detection_done
        self.icp_done_msg = msg.icp_done
        self.octomap_done_msg = msg.octomap_done# 因為這個包是直接用octomap server2的, 所以只有它是在之後nbv的時候會被改著定義
        self.nbv_done_msg = msg.nbv_done
        self.nbv_point_x_msg = msg.nbv_point_x
        self.nbv_point_y_msg = msg.nbv_point_y
        self.nbv_point_z_msg = msg.nbv_point_z
        self.nbv_point_rx_msg = msg.nbv_point_rx
        self.nbv_point_ry_msg = msg.nbv_point_ry
        self.nbv_point_rz_msg = msg.nbv_point_rz
        self.is_final_result_msg = msg.is_final_result
        self.arm_move_done_status_msg = msg.arm_move_done_status

        self.doneReset = True

    def status_reset(self): #初始化所有參數, 再把iteration設為0（表示前一個iteration已完成, 可執行下移步驟後, 各個node才會接續著跟著動作）
          
        msg_status = NodeStatus()
        msg_status.ready_for_next_iteration = True
        msg_status.target_box_id = 0
        msg_status.is_moving = False
        msg_status.iteration = 0
        msg_status.detection_done = False
        msg_status.icp_done = False
        msg_status.octomap_done = False# 因為這個包是直接用octomap server2的, 所以只有它是在之後nbv的時候會被改著定義
        msg_status.nbv_done = False
        msg_status.nbv_point_x = 0.0 #這當預設的反正這個到時候是base也不可能
        msg_status.nbv_point_y = 0.0
        msg_status.nbv_point_z = 0.0
        msg_status.nbv_point_rx = 0.0 #這當預設的反正這個到時候是base也不可能
        msg_status.nbv_point_ry = 0.0
        msg_status.nbv_point_rz = 0.0
        msg_status.is_final_result = False
        msg_status.arm_move_done_status=False
        self.publish_status_.publish(msg_status)

        self.nbv_results = []
        self.time_results = []
        self.get_logger().info('Reset Status1 done') # CHANGE

        
        # self.doneReset = True
    def status_reset_within_iteration(self): #初始化所有參數, 再把iteration設為0（表示前一個iteration已完成, 可執行下移步驟後, 各個node才會接續著跟著動作）
        msg_status = NodeStatus()
        msg_status.ready_for_next_iteration = True
        msg_status.target_box_id = self.target_box_id_msg
        msg_status.is_moving = self.is_moving_msg
        msg_status.iteration = self.iteration_msg+1
        msg_status.detection_done = False
        msg_status.icp_done = False
        msg_status.octomap_done = False# 因為這個包是直接用octomap server2的, 所以只有它是在之後nbv的時候會被改著定義
        msg_status.nbv_done = False
        msg_status.nbv_point_x = self.nbv_point_x_msg #這當預設的反正這個到時候是base也不可能
        msg_status.nbv_point_y = self.nbv_point_y_msg
        msg_status.nbv_point_z = self.nbv_point_z_msg
        msg_status.nbv_point_rx = self.nbv_point_rx_msg #這當預設的反正這個到時候是base也不可能
        msg_status.nbv_point_ry = self.nbv_point_ry_msg
        msg_status.nbv_point_rz = self.nbv_point_rz_msg
        msg_status.is_final_result = self.is_final_result_msg
        msg_status.arm_move_done_status = self.arm_move_done_status_msg
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

                self.time_start = time.perf_counter() #time.time()
            else: 
                self.status_reset_within_iteration()
            


            #1. 要iteration_msg==0
            temp_time0_iter_start = time.perf_counter()
            while(self.doneReset!=True): #會一直等直到被初始化完成
                continue
            while(self.ready_for_next_iteration_msg!=True): #會一直等直到被初始化完成
                continue
            
            self.get_logger().info("Done initializing this scene")
            temp_time1_init_end = time.perf_counter()
            
            self.get_logger().info("start for the "+str(self.iteration_msg)+"th iteration")
            
            

            #2. 等待完成detection =========================================
            self.get_logger().info("Doing the detection for tomato...")
            while(self.detection_done_msg != True): #等待完成
                continue
            self.get_logger().info("Done Detection")
            temp_time2_detect_end = time.perf_counter()
            
            self.get_logger().info("Target Box ID = "+str(self.target_box_id_msg))

            
            
            #3. 等待完成filter & ICP  ======================================
            self.get_logger().info("Doing the filter & ICP for tomato...")
            while(self.icp_done_msg != True): #等待完成ICP
                continue  
            self.get_logger().info("Done filter & ICP")
            temp_time3_filter_end = time.perf_counter()
            

            #4. 等待完成Octomap  ======================================
            self.get_logger().info("Doing the pcd to Octomap convertion...")
            while(self.octomap_done_msg != True): #等待完成ICP
                continue  
            self.get_logger().info("Done Octomap")
            temp_time4_octomap_end = time.perf_counter()
            

            #5. 等待完成NBV calculation  ======================================
            self.get_logger().info("Calculating NBV toward target tomato...")
            while(self.nbv_done_msg != True): #等待完成ICP
                continue  
            self.get_logger().info("Done Calculating NBV toward target tomato")
            temp_time5_nbv_end = time.perf_counter()
            
            self.get_logger().info("============== Result =============")
            self.get_logger().info("Iteration: "+str(self.iteration_msg)+"th")
            # self.get_logger().info("The NBV for the tomato is: ( %.2f, %.2f, %.2f)",(self.nbv_point_x_msg),(self.nbv_point_y_msg), (self.nbv_point_z_msg))
            self.get_logger().info(f"The NBV for this tomato is: ( {self.nbv_point_x_msg:.2f}, {self.nbv_point_y_msg:.2f}, {self.nbv_point_z_msg:.2f} )")
            self.get_logger().info(f"The Orientation is: ( {self.nbv_point_rx_msg:.2f}, {self.nbv_point_ry_msg:.2f}, {self.nbv_point_rz_msg:.2f} )")
            self.get_logger().info("===================================")
            self.nbv_results.append([self.nbv_point_x_msg, self.nbv_point_y_msg, self.nbv_point_z_msg, self.nbv_point_rx_msg, self.nbv_point_ry_msg, self.nbv_point_rz_msg])
            if(self.is_final_result_msg==False): 
                self.get_logger().info("SENDINT MOVING COMMAND TO ROBOT ARM NODE...")
                
                while(self.arm_move_done_status_msg == 0): #等待robot arm 移動完, 如果還沒移動完就會是0, 就會卡在這個while loop
                    continue  
                # self.get_logger().info("MOVING THE ROBOT ARM...")
                # while(self.is_moving_msg != False): #等待移動好變回靜止
                #     continue  
                if (self.arm_move_done_status_msg == 1):
                    self.get_logger().info("SUCCESSFULLY MOVE CAMERA TO BEST VIEW POSITION")
                elif (self.arm_move_done_status_msg == 2):
                    self.get_logger().info("This nbv point is unreachable, needs to recalculate the nbv point")
                


                
                self.doneReset=False
                # self.status_reset_within_iteration() #didn't reset iteration... x, y, z
                self.get_logger().info("Done Reset starting next iteration")
                # ready for...
                temp_time6_arm_end = time.perf_counter()
            
                self.time_results.append([
                    temp_time1_init_end - temp_time0_iter_start,
                    temp_time2_detect_end - temp_time1_init_end,
                    temp_time3_filter_end - temp_time2_detect_end,
                    temp_time4_octomap_end - temp_time3_filter_end,
                    temp_time5_nbv_end - temp_time4_octomap_end,
                    temp_time6_arm_end - temp_time5_nbv_end, 
                ])
            else: 
                self.get_logger().info("Arrive the final position for grabbing")
                self.get_logger().info(f"Robot arm now position (x, y, z, Rx, Ry, Rz): ({self.nbv_point_x_msg:.2f}, {self.nbv_point_y_msg:.2f}, {self.nbv_point_z_msg:.2f}, {self.nbv_point_rx_msg:.2f}, {self.nbv_point_ry_msg:.2f}, {self.nbv_point_rz_msg:.2f})")
                # self.status_reset() #不可以在這裡reset, 要等下令
                # self.get_logger().info("Done Reset, starting NBV for next Target Tomato scene")
                self.ready_for_new_Tomato = True
                temp_time6_arm_end = time.perf_counter()
            
                self.time_results.append([
                    temp_time1_init_end - temp_time0_iter_start,
                    temp_time2_detect_end - temp_time1_init_end,
                    temp_time3_filter_end - temp_time2_detect_end,
                    temp_time4_octomap_end - temp_time3_filter_end,
                    temp_time5_nbv_end - temp_time4_octomap_end,
                    temp_time6_arm_end - temp_time5_nbv_end, 
                ])
                
                # user_input = input("Enter 'n' to start NBV process for next target tomato: ").strip()
                print("============ [Final Report For this Scene] ============")
                step_names = ["Init", "Detection", "Filter", "Octomap", "NBV", "Arm"]
                scene_total_time = 0.0
                for idx, (time_result, nbv_result) in enumerate(zip(self.time_results, self.nbv_results)):
                    print(f"iteration {idx}:")
                    for step_name, time_used in zip(step_names, time_result):
                        print(f"  {step_name:<10}: {time_used:.6f} sec") # <Note> <10 靠左對齊格式
                    iter_total_time = sum(time_result)
                    scene_total_time += iter_total_time
                    print(f"  Total time for iteration: {iter_total_time:.6f} sec")
                    print(f"NBV for iteration: ({nbv_result[0]:.2f}, {nbv_result[1]:.2f}, {nbv_result[2]:.2f}, {nbv_result[3]:.2f}, {nbv_result[4]:.2f}, {nbv_result[5]:.2f})")
                    print()
                print(f"Total time for this scene: {scene_total_time:.6f} sec")

                print("=====================================================")
                
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


