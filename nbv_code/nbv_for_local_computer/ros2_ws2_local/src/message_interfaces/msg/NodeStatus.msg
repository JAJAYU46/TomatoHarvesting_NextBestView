# will record the ismoving of the camera
bool ready_for_next_iteration
bool is_moving  # <Debug> msg field 要is_moving才行, 不可isMoving
int32 target_box_id
int32 iteration #start from 1 (when 0 means it is done with this tomato nbv)
bool detection_done
bool icp_done
bool octomap_done# 因為這個包是直接用octomap server2的, 所以只有它是在之後nbv的時候會被改著定義
bool nbv_done
float64 nbv_point_x
float64 nbv_point_y
float64 nbv_point_z
float64 nbv_point_rx
float64 nbv_point_ry
float64 nbv_point_rz
bool is_final_result
int8 arm_move_done_status # 用來紀錄moving robot arm的成功狀態 # 0: 還沒move, 1: move 完了 2: unreachable nbv point, recalculate nbv point
# bool 
# int32 lu_y  # Top-left corner y-coordinate
# int32 rd_x  # Bottom-right corner x-coordinate
# int32 rd_y  # Bottom-right corner y-coordinate

