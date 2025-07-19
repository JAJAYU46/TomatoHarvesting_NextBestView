#!/user/bin/env python3 
'''
這邊的才是最新的(20250506)
[arm_move_controller]
1. Introduction: 
This Node is a movement controller for Robot Arm. It will subscribe the data from /nbv/status_communicator topic 
when the next-best-view point is calculated, do some reachability checking and send the moving command to the 
robot arm. Also, it will send whether the robot arm is moving to the /nbv/status_communicator msg.is_moving.

2. Input and Output: 



---
# <Section> For controlling TF frame for gazebo vs real world
        if(INPUT_MODE==1 or INPUT_MODE==2): # gazebo
'''

import rclpy #library for ROS2
from rclpy.node import Node
# from geometry_msgs.msg import Twist #For publisher

# for nbv/status_controller topic
from message_interfaces.msg import NodeStatus    # CHANGE

# use numpy array to store the nbv point can be better when dealing with numerical stuff
import numpy as np

# For Robot Arm Control
try:
    from tm_msgs.msg import *
    from tm_msgs.srv import *
except ImportError as e:
    tm_msgs_available = False 
    print(f"ImportError: {e}")
else:
    tm_msgs_available = True
# from move_tm.tmr_utils import TMRUtils
# tmr = TMRUtils()

IS_SOLO_NODE = False #當只有這個node開發階段時用
Open_REAL_ARM_CONTROLL = True # 預設是要下面用args去控制如果是在gazebo測試時, 就不用

import sys
INPUT_MODE=0 #1. gazebo big tomato 2. gazebo small tomato 3. realsense


class MyNode(Node): #construct Node class
    def __init__(self): #construct constructor
        super().__init__("arm_move_controller") #set python_NodeName


        
        
        # [subscriber]
        # Subscribe from /nbv/status_communicator topic
        self.status_subscription = self.create_subscription(NodeStatus, '/nbv/status_communicator',self.status_callback,10)
        
        #【publisher】
        # self.cmd_vel_pub_=self.create_publisher(Twist, "/turtle/cmd_vel", 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        self.publish_status_=self.create_publisher(NodeStatus, "/nbv/status_communicator", 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        # timer
        self.create_timer(1.0, self.callback1) #(time interval/ calling callback)
        if(IS_SOLO_NODE==True):
            self.create_timer(1.0, self.setFakeStatusForSoloNode_callback)
		
        # initialization
        self.is_moving_msg = False
        self.renew_done=False

        # Important Variables
        self.Recieved_nbv_point = np.zeros(6)
        #【subscriber】
        # self.pose_subscriber_=self.create_subscription(Pose, "/turtle/cmd_vel", self.callback_forSubscribe, 10) #(messageType/ "Topic_Name"/ callbackName/ Queue size)
        
        # For Robot Arm Control
        if (Open_REAL_ARM_CONTROLL):
            self.arm_client_ = self.create_client(SendScript, 'send_script')
            while not self.arm_client_.wait_for_service(timeout_sec = 1.0):
                self.get_logger().info('service not availabe, waiting again...')
         





        self.get_logger().info("arm_move_controller have been started")

           

        
        # self.create_timer(1.0, self.callback1) #(time interval/ calling callback)
    def setFakeStatusForSoloNode_callback(self):
        self.ready_for_next_iteration_msg = True
        self.is_moving_msg = False
        self.target_box_id_msg = 0
        self.iteration_msg = 0
        self.detection_done_msg = True
        self.icp_done_msg =True
        self.octomap_done_msg = True # 因為這個包是直接用octomap server2的, 所以只有它是在之後nbv的時候會被改著定義
        self.nbv_done_msg = True
        self.nbv_point_x_msg = 300
        self.nbv_point_y_msg = 0
        self.nbv_point_z_msg = 500
        self.nbv_point_rx_msg = 0
        self.nbv_point_ry_msg = 90
        self.nbv_point_rz_msg = 0
        self.is_final_result_msg = False
        self.arm_move_done_status_msg = False
        
        
        self.renew_done = True
        self.Recieved_nbv_point[:]=[self.nbv_point_x_msg, self.nbv_point_y_msg, self.nbv_point_z_msg, self.nbv_point_rx_msg, self.nbv_point_ry_msg, self.nbv_point_rz_msg]

    def status_callback(self,msg):
        if (self.is_moving_msg != msg.is_moving): 
            self.get_logger().info('now the is_moving_msg: '+str(msg.is_moving)) # CHANGE
        self.ready_for_next_iteration_msg = msg.ready_for_next_iteration
        self.is_moving_msg = msg.is_moving
        self.target_box_id_msg = msg.target_box_id
        self.iteration_msg = msg.iteration
        self.detection_done_msg = msg.detection_done
        self.icp_done_msg = msg.icp_done
        self.octomap_done_msg = msg.octomap_done # 因為這個包是直接用octomap server2的, 所以只有它是在之後nbv的時候會被改著定義
        self.nbv_done_msg = msg.nbv_done
        # self.nbv_point_x_msg = msg.nbv_point_x
        # self.nbv_point_y_msg = msg.nbv_point_y
        # self.nbv_point_z_msg = msg.nbv_point_z
        # self.nbv_point_rx_msg = msg.nbv_point_rx
        # self.nbv_point_ry_msg = msg.nbv_point_ry
        # self.nbv_point_rz_msg = msg.nbv_point_rz
        self.is_final_result_msg = msg.is_final_result
        self.arm_move_done_status_msg = msg.arm_move_done_status
        
        
        self.renew_done = True
        self.Recieved_nbv_point[:]=[(msg.nbv_point_x*1000), (msg.nbv_point_y*1000), (msg.nbv_point_z*1000), msg.nbv_point_rx, msg.nbv_point_ry, msg.nbv_point_rz]
        # [:] 變成是更改已存在的array而非replace it, 會更efficient
        
    def callback1(self): #construct a callback

        if (self.renew_done==True): 
            if(self.nbv_done_msg==True and self.arm_move_done_status_msg == 0): # 如果做完nbv了, 且還沒移動手臂就要把這個收到的座標紀錄印出來
                self.get_logger().info('arm_move_controller node Recieve Next-Best-View point') # CHANGE
                # self.get_logger().info(f'Recieved NBV point: ({self.nbv_point_x_msg:2f}, {self.nbv_point_y_msg:2f}, {self.nbv_point_z_msg:2f}, {self.nbv_point_rx_msg:2f}, {self.nbv_point_ry_msg:2f}, {self.nbv_point_rz_msg:2f})') # CHANGE
                self.get_logger().info(f'Recieved NBV point: ({self.Recieved_nbv_point[0]:2f}, {self.Recieved_nbv_point[1]:2f}, {self.Recieved_nbv_point[2]:2f}, {self.Recieved_nbv_point[3]:2f}, {self.Recieved_nbv_point[4]:2f}, {self.Recieved_nbv_point[5]:2f})') # CHANGE
                
                if(self.is_final_result_msg == True): #如果已經到達最佳點, 表示上次就到了, 那就不用再重新移動了
                    self.get_logger().info('Already reach the final best view point')
                    self.arm_move_done_status_msg = 1
                    self.change_nbv_status_topic(self.ready_for_next_iteration_msg, self.target_box_id_msg, False, self.iteration_msg, self.icp_done_msg, self.octomap_done_msg, self.nbv_done_msg, self.Recieved_nbv_point, self.is_final_result_msg, self.arm_move_done_status_msg)
                            
                else:
                    if(self.Is_Point_Reachable_By_RArm()): # 如果true --> reachable
                        user_input = input("Confirm Sending Command to Robot Arm?? (Y/ N): ").strip()
                        if(user_input=='Y' or user_input=='y'):
                            # self.change_nbv_status_topic(self.ready_for_next_iteration_msg, self.target_box_id_msg, self.is_moving_msg, self.iteration_msg, self.icp_done_msg, self.octomap_done_msg, self.nbv_done_msg, self.Recieved_nbv_point, self.is_final_result_msg, self.arm_move_done_status_msg)
                            
                            self.change_nbv_status_topic(self.ready_for_next_iteration_msg, self.target_box_id_msg, True, self.iteration_msg, self.icp_done_msg, self.octomap_done_msg, self.nbv_done_msg, self.Recieved_nbv_point, self.is_final_result_msg, self.arm_move_done_status_msg)
                            
                            # self.change_is_moving_status(True)

                            self.send_MovingCommandToArm()

                            while(True): 
                                print("waiting for robot arm done moving...")
                                user_input2 = input("Does the robot arm done moving?").strip()
                                if(user_input2=='Y' or user_input2=='y'):
                                    break
                            
                            self.get_logger().info('Successfully moving to the new robor arm coordinate') # CHANGE

                            self.arm_move_done_status_msg = 1
                            self.change_nbv_status_topic(self.ready_for_next_iteration_msg, self.target_box_id_msg, False, self.iteration_msg, self.icp_done_msg, self.octomap_done_msg, self.nbv_done_msg, self.Recieved_nbv_point, self.is_final_result_msg, self.arm_move_done_status_msg)
                            
                            # self.change_is_moving_status(False)
                        else: 
                            self.get_logger().info('Not Sending Moving Command to Robot Arm') # CHANGE
                            self.get_logger().info('Need to recalculate the next-best-view for this scene')
                            self.arm_move_done_status_msg = 2 # 先改好自己再改大家
                            # self.is_final_result_msg = False #因為這個點被否決, 表示他不是最好的點了
                            self.change_nbv_status_topic(self.ready_for_next_iteration_msg, self.target_box_id_msg,self.is_moving_msg, self.iteration_msg, self.icp_done_msg, self.octomap_done_msg, self.nbv_done_msg, self.Recieved_nbv_point, self.is_final_result_msg, self.arm_move_done_status_msg) # 2 means it is out of range or need to recalculate the nbv value
                            
                    else: 
                        self.get_logger().info('The Next-Best-View is unreachable for the robot arm') # CHANGE
                        self.get_logger().info('Need to recalculate the next-best-view for this scene') # CHANGE
                        self.arm_move_done_status_msg = 2
                        self.change_nbv_status_topic(self.ready_for_next_iteration_msg, self.target_box_id_msg,self.is_moving_msg, self.iteration_msg, self.icp_done_msg, self.octomap_done_msg, self.nbv_done_msg, self.Recieved_nbv_point, self.is_final_result_msg, self.arm_move_done_status_msg) # 2 means it is out of range or need to recalculate the nbv value
                            
                
                


                # user_input = input("Enter 'n' to start NBV process for next target tomato: ").strip()
    # ================== Function For Robot Arm Control ==================
    
    def send_script(self, script):
      move_cmd = SendScript.Request()
      move_cmd.script = script
      self.arm_client_.call_async(move_cmd)  
    
    def Cmotions(self, cmd_Cpose):
        for i in range(len(cmd_Cpose)):
            self.MoveCartesian(cmd_Cpose[i])
    
    def MoveCartesianVelocity(self, Cpvt):
            self.send_script("PVTPoint(" + self.ToStr(Cpvt) + ")")
    def MoveCartesian(self, Cpose):
        self.send_script("PTP(\"CPP\"," + self.ToStr(Cpose) + ",6,500,100,false)") # TCP Speed = 55 [mm/s] / 5 %, 110 [mm/s] / 10 %
    def ToStr(self, array):
        return ", ".join(str(array_num) for array_num in array)
    
    # ====================================================================            
                
                
    def Is_Point_Reachable_By_RArm(self): 
        if (self.Recieved_nbv_point[2]<=0): #如果跑到z<0就是unreachable
            return False
        return True

    def send_MovingCommandToArm(self):
        self.get_logger().info('SENDING MOVING COMMAND TO ROBOT ARM...') # CHANGE
        self.get_logger().info(f'MOVING ROBOT ARM TO: ({self.Recieved_nbv_point[0]:2f}, {self.Recieved_nbv_point[1]:2f}, {self.Recieved_nbv_point[2]:2f}, {self.Recieved_nbv_point[3]:2f}, {self.Recieved_nbv_point[4]:2f}, {self.Recieved_nbv_point[5]:2f})') # CHANGE
        
        
        if (Open_REAL_ARM_CONTROLL): 
            goal_command = np.array([self.Recieved_nbv_point[0], self.Recieved_nbv_point[1], self.Recieved_nbv_point[2], int(self.Recieved_nbv_point[3]), int(self.Recieved_nbv_point[4]), int(self.Recieved_nbv_point[5])])#np.array([500, 0,  500,  0,   90, 0]) #np.array(self.Recieved_nbv_point[0:5])
            # self.MoveCartesian(goal_command)
            cmd_Cpose = [
                    goal_command,
                    ]
            self.Cmotions(cmd_Cpose)
        
        # self.get_logger().info('Successfully moving to the new robor arm coordinate') # CHANGE

                        
    def change_nbv_status_topic(self, ready_for_next_iteration, target_box_id, is_moving, iteration, icp_done, octomap_done, nbv_done, Recieved_nbv_point, is_final_result, arm_move_done_status): #初始化所有參數, 再把iteration設為0（表示前一個iteration已完成, 可執行下移步驟後, 各個node才會接續著跟著動作）
          
        msg_status = NodeStatus()
        msg_status.ready_for_next_iteration = ready_for_next_iteration
        msg_status.target_box_id = target_box_id
        msg_status.is_moving = is_moving
        msg_status.iteration = iteration
        msg_status.detection_done = self.detection_done_msg
        msg_status.icp_done = icp_done
        msg_status.octomap_done = octomap_done
        msg_status.nbv_done = nbv_done
        msg_status.nbv_point_x = Recieved_nbv_point[0]
        msg_status.nbv_point_y = Recieved_nbv_point[1]
        msg_status.nbv_point_z = Recieved_nbv_point[2]
        msg_status.nbv_point_rx = Recieved_nbv_point[3]
        msg_status.nbv_point_ry = Recieved_nbv_point[4]
        msg_status.nbv_point_rz = Recieved_nbv_point[5]
        msg_status.is_final_result = is_final_result
        msg_status.arm_move_done_status = arm_move_done_status
        
        self.publish_status_.publish(msg_status)
        # self.get_logger().info('Changing moving status to: '+str(NewIsMoving)) 
                


    # def change_is_moving_status(self, NewIsMoving): #初始化所有參數, 再把iteration設為0（表示前一個iteration已完成, 可執行下移步驟後, 各個node才會接續著跟著動作）
          
    #     msg_status = NodeStatus()
    #     msg_status.ready_for_next_iteration = self.ready_for_next_iteration_msg
    #     msg_status.target_box_id = self.target_box_id_msg
    #     msg_status.is_moving = NewIsMoving
    #     msg_status.iteration = self.iteration_msg
    #     msg_status.detection_done = self.detection_done_msg
    #     msg_status.icp_done = self.icp_done_msg
    #     msg_status.octomap_done = self.octomap_done_msg
    #     msg_status.nbv_done = self.nbv_done_msg
    #     msg_status.nbv_point_x = self.Recieved_nbv_point[0]
    #     msg_status.nbv_point_y = self.Recieved_nbv_point[1]
    #     msg_status.nbv_point_z = self.Recieved_nbv_point[2]
    #     msg_status.nbv_point_rx = self.Recieved_nbv_point[3]
    #     msg_status.nbv_point_ry = self.Recieved_nbv_point[4]
    #     msg_status.nbv_point_rz = self.Recieved_nbv_point[5]
    #     msg_status.is_final_result = self.is_final_result_msg
    #     self.publish_status_.publish(msg_status)
    #     self.get_logger().info('Changing moving status to: '+str(NewIsMoving)) 
                


       
        
def main(args=None): #construct main function
    rclpy.init(args=args)

    global INPUT_MODE, Open_REAL_ARM_CONTROLL
    if len(sys.argv) > 1:
        INPUT_MODE = int(sys.argv[1])  # Get the argument from the command line
    else:
        INPUT_MODE = 3  # Default value  # default value if no args are provided  #1. gazebo big tomato 2. gazebo small tomato 3. realsense
    # run with 'ros2 run my_robot_nbv nbv_tompcd_filter 2'
    print("INPUT_MODE:", INPUT_MODE)

    # <Section> For controlling TF frame for gazebo vs real world
    if(INPUT_MODE==1 or INPUT_MODE==2): # gazebo
        Open_REAL_ARM_CONTROLL = False
    node1 = MyNode() #node1=NodeClass: MyNode
    rclpy.spin(node1) #keep node alive until ctrl+C
    rclpy.shutdown()

if __name__=='__main__':
        main()	
