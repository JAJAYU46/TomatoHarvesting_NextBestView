import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import copy

# INPUT_MODE=1 #1. gazebo big tomato 2. gazebo small tomato 3. realsense

IsDebug = False
IsDebug_Simple = False

#Functions
#[For 預處理]
#用來畫出平移..之後的點雲
source_down_global = None
target_down_global = None
def draw_registration_result(source, target, transformation, window_name="open3D"):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    # o3d.visualization.draw_plotly([source_temp, target_temp])
    
    # Create a visualizer object
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name)
    
    # Add the point clouds
    vis.add_geometry(source_temp)
    vis.add_geometry(target_temp)
    
    # Add coordinate frame for reference
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.001))
   
    render_option = vis.get_render_option()
    render_option.point_size = 0.001  # Adjust this value to change the point size
    
    vis.run()
    vis.destroy_window()

#[For Global Registration]
def preprocess_point_cloud(pcd, voxel_size):
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    # pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    
    if IsDebug:
        print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    if IsDebug:
        print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd, pcd_fpfh

#給sample跟target pcd, 跟sample跟target的fpfh資料, 會回傳一個含有transformation 矩陣的result(By用result.transformation)
def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    if IsDebug:
        print(":: RANSAC registration on downsampled point clouds.")
        print("   Since the downsampling voxel size is %.3f," % voxel_size)
        print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength( #兩個找轉換矩陣的功能方法, 這個在找source跟target的edge長度是不是0.9倍
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance( #這個在找兩個點之間的距離...的
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    if IsDebug:
        print(result.transformation)
    return result


#================== ICP ====================
def ICPoperation(source, target, TomatoBox_lu=[0,0], TomatoBox_rd=[10000,10000], projection_para = [610, 610, 345, 237.085205078125], camera_point = np.array([0,0,0])):
    # vis = o3d.visualization.Visualizer()
    # vis.create_window(window_name="Tomato Registration Process")
    # vis.run()
    if IsDebug:
        draw_registration_result(source, target, np.identity(4), "original pose")#先畫一次轉換前 "original pose": window_name for visualization
    #[global registration]
    voxel_size=0.001#for tomato
    #< b. 做預先處理> 得到down sampling的pcd跟sample跟target的fpfh資料
    source_down_origin, source_fpfh_origin = preprocess_point_cloud(source, voxel_size) #已經做過down sampling 的pcd, 就是現在在用的pcd 叫做source_down了
    target_down_origin, target_fpfh_origin = preprocess_point_cloud(target, voxel_size)

    #因為...一直大於一, 所以就試試copy一份新的看看
    # Before the first call
    global source_down_global, target_down_global,  result_icp_final_global
    source_down = copy.deepcopy(source_down_origin)
    target_down = copy.deepcopy(target_down_origin)
    source_fpfh = copy.deepcopy(source_fpfh_origin)
    target_fpfh = copy.deepcopy(target_fpfh_origin)

    source_down_global = copy.deepcopy(source_down)
    target_down_global = copy.deepcopy(target_down)


    #< c. 得到轉換矩陣>（target轉成sample orientation的轉換矩陣（存成result_ransac.transformation））
    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,voxel_size)

    if IsDebug:
        # print("Check if ICP acceptable: ", check_if_icp_acceptable(pcd_icp, pcd_real, camera_point, TomatoBox_lu, TomatoBox_rd), "RedoGlobal Count = ", RedoGlobalCount" )
        print("Check if ICP acceptable: ", check_if_icp_acceptable(source_down.transform(result_ransac.transformation), target_down, camera_point, TomatoBox_lu, TomatoBox_rd))
        draw_registration_result(source_down, target_down, result_ransac.transformation, "Global registration")

    
    #==================Evaluation如果“Global準度沒達到多少, 就重算global==========================
    threshold=0.001
    source_num_points = len(source_down.points)
    target_num_points = len(target_down.points)
    # print(target_num_points) #共108點, 成功的是有71個點, 即72％的點都有被配對到
    # print(target_num_points/source_num_points)
    sp_tp_ratio=target_num_points/source_num_points

    evaluation = o3d.pipelines.registration.evaluate_registration(source_down, target_down, threshold, result_ransac.transformation)
    if IsDebug:
        print("global registration evaluation: "+str(evaluation))
        # print(len(evaluation.correspondence_set))
        print("target_num_points=%d" %target_num_points)
        print("paired_point_num=%d" %len(evaluation.correspondence_set))
    paired_point_ratio=len(evaluation.correspondence_set)/target_num_points #target中有幾趴的點又被配對到, 配對到的點太少就要重算global registration
    
    paired_point_ratio_largestNow=0
    result_ransac_largestNow = None
    Is_icp_acceptable = check_if_icp_acceptable(copy.deepcopy(source_down).transform(result_ransac.transformation), target_down, camera_point, TomatoBox_lu, TomatoBox_rd, projection_para)
    if (Is_icp_acceptable == True and result_ransac is not None): #可接受才存 
        result_ransac_largestNow = result_ransac #紀錄corresponding point ratio最大者 (所以如果超過5次還沒能到達標準, 就取這5次中corresponding point ratio最大者)
        paired_point_ratio_largestNow=paired_point_ratio
    
    if IsDebug:
        print("corresponding point= %.2f" %paired_point_ratio)
        print("good_fitness_standard= %.2f" %((sp_tp_ratio)*0.6))
    
    RedoGlobalCount = 0 #<Debug6>用來計算global registration到底重做了幾次, 如果超過3次就代表真的就是correspondent point太少, 就直接不做了, 跳出來才部會卡在迴圈
    AcceptRedoGlobalCount = 10#3

    print("sp_tp_ratio=%.2f" %sp_tp_ratio)
    print("corresponding point= %.2f" %paired_point_ratio)
    print("global registration evaluation: "+str(evaluation))
    # GlobalRegistrationSuccessFlag = True
    # while (paired_point_ratio<=0.60) or (evaluation.inlier_rmse > 0.01) or (evaluation.fitness<(sp_tp_ratio)*0.6): #如果global誤差太大, 就重算global #這個的好像標準太高了會一直卡在這個迴圈
    # while (paired_point_ratio<=0.60): #如果global誤差太大, 就重算global

        
    # while (paired_point_ratio<=0.35) or (sp_tp_ratio<=0.04): #如果global誤差太大, 就重算global #20250108
    while (paired_point_ratio<0.83) or (evaluation.inlier_rmse > 0.11) or (evaluation.fitness<(sp_tp_ratio)*0.6) or (Is_icp_acceptable == False):
    
        # evaluation.inlier_rmse 正常的時候是0.00762, 如果這次算出的global誤差太大, 就重算global
        #不知為啥有時paired_point_ratio會大於1ㄟ其實不太對的
        if RedoGlobalCount<=AcceptRedoGlobalCount: #如果是None也沒關係, 就代表現在還沒抓到對的蕃茄位置, 就回傳none重新抓 or result_ransac_largestNow == None: #如果太多次了或是就是還沒有一個Acceptable的話，就繼續找
            if IsDebug:
                print("recalculate global registration (target_num_points/len(evaluation.correspondence_set))<0.60) or (evaluation.inlier_rmse > 0.01)")
            #===========test=============
            # source_down = copy.deepcopy(source_down_origin)
            # target_down = copy.deepcopy(target_down_origin)
            # source_fpfh = copy.deepcopy(source_fpfh_origin)
            # target_fpfh = copy.deepcopy(target_fpfh_origin)
            #============================
            result_ransac = execute_global_registration(source_down, target_down,
                                                        source_fpfh, target_fpfh,voxel_size)
    
            evaluation = o3d.pipelines.registration.evaluate_registration(source_down, target_down, threshold, result_ransac.transformation)
            paired_point_ratio=len(evaluation.correspondence_set)/target_num_points
            print("sp_tp_ratio=%.2f" %sp_tp_ratio)
            print("corresponding point= %.2f" %paired_point_ratio)
            print("global registration evaluation: "+str(evaluation))
            if IsDebug:
                print("corresponding point= %.2f" %paired_point_ratio)
                print("global registration evaluation: "+str(evaluation))
            if IsDebug:
                # print("Check if ICP acceptable: ", check_if_icp_acceptable(pcd_icp, pcd_real, camera_point, TomatoBox_lu, TomatoBox_rd), "RedoGlobal Count = ", RedoGlobalCount" )
                # print("Check if ICP acceptable: ", check_if_icp_acceptable(source_down.transform(result_ransac.transformation), target_down, camera_point, TomatoBox_lu, TomatoBox_rd))
                draw_registration_result(source_down, target_down, result_ransac.transformation, "ReGlobal_registration")
            # <Debug> 注意transform() modifies source_down in-place, 所以如果在傳進函數隻前transform好就會改到source_down本身!所以要用copy.deepcopy(source_down) Transformation applied twice? If draw_registration_result() also applies the transformation again, it might double-transform the source points, leading to a different visualization <ADD>    
            Is_icp_acceptable = check_if_icp_acceptable(copy.deepcopy(source_down).transform(result_ransac.transformation), target_down, camera_point, TomatoBox_lu, TomatoBox_rd, projection_para)
            if IsDebug:
                print("Is_icp_acceptable: ", Is_icp_acceptable)
                print(" =================================================================== ")
            if(paired_point_ratio>paired_point_ratio_largestNow and (Is_icp_acceptable == True and result_ransac is not None)): #可接受才存 ):
                result_ransac_largestNow = result_ransac
                paired_point_ratio_largestNow = paired_point_ratio
            

            RedoGlobalCount+=1
        else: #如果重作太多次了就跳出迴圈 , 就直接回傳None 跳出function 了
            # return None
            result_ransac=result_ransac_largestNow
            print("Do To many times, is result_ransac_largestNow")
            print(" =================================================================== ")
            break #看看如果global沒取好照做會怎樣 #結果好像也是可以, 你可能上面標準設太高了
            
    # if(RedoGlobalCount>)
    if result_ransac == None:
        print("global registration fail due to too many try, return none and reget the bounding box")
        return result_ransac

    #[Step2] Local Registration (By ICP method)
    icp_count=10
    result_icp_final=ICP_helper(source, target, result_ransac, icp_count, threshold)
    result_icp_final_global = result_icp_final
    if IsDebug:
        print("Total "+str(icp_count)+"th time of point-to-point ICP")
    if True:
        if IsDebug_Simple: 
            draw_registration_result(source_down, target_down, result_ransac.transformation, "Global_registration(ICP)")
            draw_registration_result(source_down, target_down, result_icp_final.transformation, "Local_registration(ICP)")
    if IsDebug:
        print("The final transformation matrics is: ")
        print(result_icp_final.transformation)  

    print("Done ICP and Everything")
    return result_icp_final  
    
    
#ICP部份用recursive寫
def ICP_helper(source, target, global_result, count, threshold):
    #中止條件
    if count<=0:
        return global_result #在最最最內層, 要先從global trans開始
    count -=1
    #下面是open3d內建的icp方法, 可以得到local下的轉換矩陣
    local_result=ICP_helper(source, target, global_result, count, threshold)
    new_local_result = o3d.pipelines.registration.registration_icp(source, target, threshold, local_result.transformation,o3d.pipelines.registration.TransformationEstimationPointToPoint())
    if IsDebug:
        print(new_local_result.transformation)
    # draw_registration_result(source, target, new_local_result.transformation)
    return new_local_result

    
# ================ Function For Checking Acceptability ==================================
def find_pointcloud_centroid(points): 
    centroid = np.mean(points, axis=0)
    print("ICP Tomato centroid:", centroid)
    return centroid
    
# 1. ICP center 投影回去要在bounding box裡面
def check_icp_center_in_bbox(center, TomatoBox_lu, TomatoBox_rd, projection_para = [610, 610, 345, 237.085205078125]): 
    u, v = project_point_to_pixel_module(*center, projection_para)
    print("u, v: ", u, ", ",v)
    if TomatoBox_lu[0]<u and u<TomatoBox_rd[0]:
        if TomatoBox_lu[1]<v and v<TomatoBox_rd[1]:
            return True
    return False
    
# 2. (深度方向要對)pointcloud最近點->camera點(線在它是(0,0,0))和pointcloud最近點->icp_centroid 內積為0(即最近點要在camera點和icp centroid)中間
def pcd_close_point_is_at_middle_of_cameraAcentoid(camera_point, closest_point, icp_centroid): 
    
    # closest_point = np.array([0.5, 0, 0])
    print("The closest point is:", closest_point)
    print("Centroid: ", icp_centroid)
    print("camera_point: ", camera_point)
    # visualize_point(np.array([camera_point, closest_point, centroid]), pcd) #綠色在中間的時候是對的
    # closest_point, 
    vector_close_cam = camera_point - closest_point
    vector_close_centroid = icp_centroid - closest_point
    dot_product = np.dot(vector_close_cam, vector_close_centroid)
    if(dot_product<=0): #表示是反向，是對的
        return True
    return False

def visualize_point(points_to_print, pcds_to_print = None): 
    # Draw the centroid
    # Draw the centroid as a red sphere
    # Create a visualizer object
    vis = o3d.visualization.Visualizer()
    vis.create_window("Point Visualization")
    color = [[0, 0, 1], [0, 1, 0], [1, 0, 0]]
    color_pcd = [[1, 0.706, 0], [0, 0.651, 0.929]]
    # i = 0
    # for point in points_to_print: 
    # Loop through the points and add spheres at each point
    for i, point in enumerate(points_to_print): 
        point = np.asarray(point)
        if point.shape != (3,):
            raise ValueError(f"Each point must be a 3D coordinate, but got shape {point.shape}.")
        
        point_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.002)  # Adjust radius as needed
        point_sphere.paint_uniform_color(color[i%3])  # Red color
        point_sphere.translate(point)   # Move sphere to point
        vis.add_geometry(point_sphere)
        # i +=1 
    
    # source_for_center = copy.deepcopy(pcd_pro)
    for j, pcd in enumerate(pcds_to_print): 
        pcd.paint_uniform_color(color_pcd[j%2])
        vis.add_geometry(pcd)
    
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.001))
    render_option = vis.get_render_option()
    render_option.point_size = 1  # Adjust this value to change the point size
    vis.run()
    vis.destroy_window()


def find_pointA_to_pcd_close_point(A, pcd_points, camera_point = np.array([0,0,0])):
    # Compute the Euclidean distance between point A and each point in the point cloud
    distances = np.linalg.norm(pcd_points - camera_point, axis=1)
    # Find the index of the point with the smallest distance
    closest_point_index = np.argmin(distances)
    # Get the closest point
    closest_point = pcd_points[closest_point_index]
    return closest_point
def project_point_to_pixel_module(x, y, z, projection_para = [610, 610, 345, 237.085205078125]):#project_point_to_pixel(self, x, y, z):
    if z == 0:
        return None  # Avoid division by zero
    fx=projection_para[0]
    fy=projection_para[1]
    cx = projection_para[2]
    cy = projection_para[3]
    # Project the 3D point onto the 2D image plane
    u = fx * (x / z) + cx
    v = fy * (y / z) + cy

    return int(u), int(v)

# =========================================================================================
def check_if_icp_acceptable(pcd_icp, pcd_real, camera_point, TomatoBox_lu, TomatoBox_rd, projection_para = [610, 610, 345, 237.085205078125]):
    pcd_points_icp = np.asarray(pcd_icp.points)
    pcd_points_real = np.asarray(pcd_real.points)
    icp_centroid = find_pointcloud_centroid(pcd_points_icp)

    closest_point = find_pointA_to_pcd_close_point(camera_point, pcd_points_real)
    # closest_point = None
    if(check_icp_center_in_bbox(icp_centroid, TomatoBox_lu, TomatoBox_rd, projection_para)): 
        if(pcd_close_point_is_at_middle_of_cameraAcentoid(camera_point, closest_point, icp_centroid)): 
            print("True")
            if IsDebug_Simple: 
                visualize_point([camera_point, closest_point, icp_centroid], [pcd_icp, pcd_real])#直接用list傳(因為np.array要裡面是1D array，所以沒辦法用numpy array傳)
            return True
        else: 
            if IsDebug_Simple: 
                print("Not acceptable due to not middle")
    else: 
        if IsDebug_Simple: 
            print("Not acceptable due to not in bbox")
    if IsDebug_Simple: 
        visualize_point([camera_point, closest_point, icp_centroid], [pcd_icp, pcd_real])#直接用list傳(因為np.array要裡面是1D array，所以沒辦法用numpy array傳)
    return False





    
# ================ visualization function ==================================



# #Test Code
# #================== Start Code ====================
# # print("everything alright")
# #Import Dataset
# # source_path="/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/ros2_ws2/src/dataset/data_pcd/TomatoPlant_size_modified_only1tomato.ply"
# # target_path="/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/ros2_ws2/src/dataset/data_pcd/copy_of_filtered_msg.pcd"
# # target_path="/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/ros2_ws2/src/dataset/output_data/pointcloud/filtered_pcd_6th.ply"
# source_path="./dataset/data_pcd/TomatoPlant_only1tomato_onlyRed.ply"
# target_path="./dataset/output_data/pointcloud/filtered_pcd (8th copy).ply"
# num=3

# # demo_pcds = o3d.data.DemoICPPointClouds()
# # source = o3d.io.read_point_cloud(demo_pcds.paths[0])
# # target = o3d.io.read_point_cloud(demo_pcds.paths[1])

# source = o3d.io.read_point_cloud(source_path.format(num)) #demo_pcds.paths[0])
# target = o3d.io.read_point_cloud(target_path.format(num)) #demo_pcds.paths[1])
# print("Number of points in source:", len(source.points))
# print("Number of points in target:", len(target.points))






# import json
# # Open and read the JSON file
# with open('./dataset/output_data/bounding_box/bbox (8th copy).json', 'r') as file:
#     parsed_data = json.load(file)

# import json
# # Open and read the JSON file
# with open('./dataset/output_data/bounding_box/bbox (8th copy).json', 'r') as file:
#     parsed_data = json.load(file)
# TomatoBox_lu = [parsed_data["lu_x"], parsed_data["lu_y"]]
# TomatoBox_rd = [parsed_data["rd_x"], parsed_data["rd_y"]]
# projection_para = [610, 610, 345, 237.085205078125]
# camera_point = np.array([0,0,0])
# ICPoperation(source, target, TomatoBox_lu, TomatoBox_rd, projection_para, camera_point)

# ICPoperation(source,target)