import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import copy




#Functions
#[For 預處理]
#用來畫出平移..之後的點雲
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
    if __debug__:
        print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    if __debug__:
        print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd, pcd_fpfh

#給sample跟target pcd, 跟sample跟target的fpfh資料, 會回傳一個含有transformation 矩陣的result(By用result.transformation)
def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    if __debug__:
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
    if __debug__:
        print(result.transformation)
    return result


#================== ICP ====================
def ICPoperation(source, target):
    # vis = o3d.visualization.Visualizer()
    # vis.create_window(window_name="Tomato Registration Process")
    # vis.run()
    if __debug__:
        draw_registration_result(source, target, np.identity(4), "original pose")#先畫一次轉換前 "original pose": window_name for visualization
    #[global registration]
    voxel_size=0.001#for tomato
    #< b. 做預先處理> 得到down sampling的pcd跟sample跟target的fpfh資料
    source_down_origin, source_fpfh_origin = preprocess_point_cloud(source, voxel_size) #已經做過down sampling 的pcd, 就是現在在用的pcd 叫做source_down了
    target_down_origin, target_fpfh_origin = preprocess_point_cloud(target, voxel_size)

    #因為...一直大於一, 所以就試試copy一份新的看看
    # Before the first call
    source_down = copy.deepcopy(source_down_origin)
    target_down = copy.deepcopy(target_down_origin)
    source_fpfh = copy.deepcopy(source_fpfh_origin)
    target_fpfh = copy.deepcopy(target_fpfh_origin)


    #< c. 得到轉換矩陣>（target轉成sample orientation的轉換矩陣（存成result_ransac.transformation））
    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,voxel_size)
    if __debug__:
        draw_registration_result(source_down, target_down, result_ransac.transformation, "Global registration")

    #==================Evaluation如果“Global準度沒達到多少, 就重算global==========================
    threshold=0.001
    source_num_points = len(source_down.points)
    target_num_points = len(target_down.points)
    # print(target_num_points) #共108點, 成功的是有71個點, 即72％的點都有被配對到
    # print(target_num_points/source_num_points)
    sp_tp_ratio=target_num_points/source_num_points

    evaluation = o3d.pipelines.registration.evaluate_registration(source_down, target_down, threshold, result_ransac.transformation)
    if __debug__:
        print("global registration evaluation: "+str(evaluation))
        # print(len(evaluation.correspondence_set))
        print("target_num_points=%d" %target_num_points)
        print("paired_point_num=%d" %len(evaluation.correspondence_set))
    paired_point_ratio=len(evaluation.correspondence_set)/target_num_points #target中有幾趴的點又被配對到, 配對到的點太少就要重算global registration
    if __debug__:
        print("corresponding point= %.2f" %paired_point_ratio)
        print("good_fitness_standard= %.2f" %((sp_tp_ratio)*0.6))
    
    RedoGlobalCount = 0 #<Debug6>用來計算global registration到底重做了幾次, 如果超過3次就代表真的就是correspondent point太少, 就直接不做了, 跳出來才部會卡在迴圈
    AcceptRedoGlobalCount = 3

    print("sp_tp_ratio=%.2f" %sp_tp_ratio)
    print("corresponding point= %.2f" %paired_point_ratio)
    print("global registration evaluation: "+str(evaluation))
    # GlobalRegistrationSuccessFlag = True
    # while (paired_point_ratio<=0.60) or (evaluation.inlier_rmse > 0.01) or (evaluation.fitness<(sp_tp_ratio)*0.6): #如果global誤差太大, 就重算global #這個的好像標準太高了會一直卡在這個迴圈
    # while (paired_point_ratio<=0.60): #如果global誤差太大, 就重算global
    
    while (paired_point_ratio<=0.35) or (sp_tp_ratio<=0.04): #如果global誤差太大, 就重算global

    
        # evaluation.inlier_rmse 正常的時候是0.00762, 如果這次算出的global誤差太大, 就重算global
        #不知為啥有時paired_point_ratio會大於1ㄟ其實不太對的
        if RedoGlobalCount<=AcceptRedoGlobalCount:
            if __debug__:
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
            if __debug__:
                print("corresponding point= %.2f" %paired_point_ratio)
                print("global registration evaluation: "+str(evaluation))
            if __debug__:
                draw_registration_result(source_down, target_down, result_ransac.transformation, "ReGlobal_registration")
            RedoGlobalCount+=1
        else: #如果重作太多次了就跳出迴圈 , 就直接回傳None 跳出function 了
            # return None
            break #看看如果global沒取好照做會怎樣 #結果好像也是可以, 你可能上面標準設太高了
            


    #[Step2] Local Registration (By ICP method)
    icp_count=3
    result_icp_final=ICP_helper(source, target, result_ransac, icp_count, threshold)
    if __debug__:
        print("Total "+str(icp_count)+"th time of point-to-point ICP")
    if __debug__:
        draw_registration_result(source_down, target_down, result_icp_final.transformation, "Local_registration(ICP)")
    if __debug__:
        print("The final transformation matrics is: ")
        print(result_icp_final.transformation)  
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
    if __debug__:
        print(new_local_result.transformation)
    # draw_registration_result(source, target, new_local_result.transformation)
    return new_local_result

    
# ================ visualization function ==================================



#Test Code
#================== Start Code ====================
# print("everything alright")
#Import Dataset
# source_path="/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/ros2_ws2/src/dataset/data_pcd/TomatoPlant_size_modified_only1tomato.ply"
# target_path="/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/ros2_ws2/src/dataset/data_pcd/copy_of_filtered_msg.pcd"
# num=3

# # demo_pcds = o3d.data.DemoICPPointClouds()
# # source = o3d.io.read_point_cloud(demo_pcds.paths[0])
# # target = o3d.io.read_point_cloud(demo_pcds.paths[1])

# source = o3d.io.read_point_cloud(source_path.format(num)) #demo_pcds.paths[0])
# target = o3d.io.read_point_cloud(target_path.format(num)) #demo_pcds.paths[1])

# ICPoperation(source,target)