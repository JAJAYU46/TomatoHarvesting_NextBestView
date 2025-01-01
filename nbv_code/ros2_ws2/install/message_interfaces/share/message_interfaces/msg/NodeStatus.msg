# will record the ismoving of the camera
bool is_moving  # <Debug> msg field 要is_moving才行, 不可isMoving
int32 iteration #start from 1 (when 0 means it is done with this tomato nbv)
bool detection_done
bool icp_done
bool octomap_done# 因為這個包是直接用octomap server2的, 所以只有它是在之後nbv的時候會被改著定義
bool nbv_done
float64 nbv_point_x
float64 nbv_point_y
float64 nbv_point_z
bool is_final_result
# bool 
# int32 lu_y  # Top-left corner y-coordinate
# int32 rd_x  # Bottom-right corner x-coordinate
# int32 rd_y  # Bottom-right corner y-coordinate

