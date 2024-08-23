import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import copy

print("everything alright")


#Functions
#[For 預處理]
#[用來分別給source跟target上色的function]
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_plotly([source_temp, target_temp])


#[For Global Registration]
#[預先處理pointcloud]這個是open3d裡面附的程式用來預先處裡pointcloud可以1. downscale model 2. 得到model的normal(垂直?) 3. 得到feature, 這個feature是利用一個和左右鄰居的關係, 角度去找到的, orientation的資訊不會被算進去因為現在icp就是要把orientation對在一起
#傳進去一個pcd跟一個voxel_size(就是cube小格格的大小), 回傳一個已經經過downscale的pcd_down(現在這裡還是pcd因為沒有用down_scale)跟pcd_fpfh（他的相零point兼的角度相關資訊）
def preprocess_point_cloud(pcd, voxel_size):
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    # pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd, pcd_fpfh

#給sample跟target pcd, 跟sample跟target的fpfh資料, 會回傳一個含有transformation 矩陣的result(By用result.transformation)
#這個是machine learning 很常用的東東, goal用來降低訓練模型大小但還是保持精準度, 叫做"pruning"
#pruning: rejecting & eleminating incorrect matches to reduce the size of training by eleminating some parameters 
def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
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
    print(result.transformation)
    return result


#================== Start Code ====================

# #[Step1]  Global registration-Fast Point Feature Histograms (FPFH)
# #< a. 參數設定>
# voxel_size=0.001
# #要先有global registration(可以find "global" minimum去得到初始的轉換位置(大的移動))
# #下面事先隨便給個轉換
# # trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
# #                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
# # source.transform(trans_init)
# draw_registration_result(source, target, np.identity(4))

# #< b. 得到down sampling的pcd跟sample跟target的fpfh資料>
# #已經做過down sampling 的pcd, 就是現在在用的pcd 叫做source_down了
# source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
# target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)

# #< c. 得到轉換式（target轉成sample orientation的轉換矩陣（存成result_ransac.transformation））>
# result_ransac = execute_global_registration(source_down, target_down,
#                                             source_fpfh, target_fpfh,
#                                             voxel_size)
# print(result_ransac)
# trans_init = result_ransac.transformation #trans_init為global預測出來得出使轉換矩陣
# draw_registration_result(source_down, target_down, result_ransac.transformation)

# #[Step2] Local Registration (By ICP method)
# threshold=0.02
# print("Apply point-to-point ICP")
# #下面是open3d內建的icp方法, 可以得到local下的轉換矩陣
# reg_p2p = o3d.pipelines.registration.registration_icp(
#     source, target, threshold, trans_init,
#     o3d.pipelines.registration.TransformationEstimationPointToPoint())
# print(reg_p2p)
# print("Transformation is:")
# icp_trans=reg_p2p.transformation
# print(icp_trans)
# draw_registration_result(source, target, icp_trans)

def ICPoperation(source, target):
    
    draw_registration_result(source, target, np.identity(4))#先畫一次轉換前
    #[global registration]
    voxel_size=0.001#for tomato
    # voxel_size=0.1#for fragment.ply
    #< b. 做預先處理> 得到down sampling的pcd跟sample跟target的fpfh資料
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size) #已經做過down sampling 的pcd, 就是現在在用的pcd 叫做source_down了
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)

    #< c. 得到轉換矩陣>（target轉成sample orientation的轉換矩陣（存成result_ransac.transformation））
    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,voxel_size)
    draw_registration_result(source_down, target_down, result_ransac.transformation)

    #==================Evaluation如果“Global準度沒達到多少, 就重算global==========================
    threshold=0.001
    #沒有大篇的時候的數值
    # global registration evaluation
    # RegistrationResult with fitness=1.945739e-02, inlier_rmse=7.761717e-04, and correspondence_set size of 71
    # Access transformation to get result.
    # 0.007
    # 壞的長這樣： 
    # global registration evaluation
    # RegistrationResult with fitness=3.836668e-03, inlier_rmse=7.447310e-04, and correspondence_set size of 14
    # Access transformation to get result.
    # 成功跟失敗主要差在correspondence_set, 要夠大
    #因為target跟sample的點點術本來就差很多...
    #最好的一次（感覺差別在inlier_rmse要<0.7但很難發生, 跟fitness要大到0.02了）
    # global registration evaluation
    # RegistrationResult with fitness=2.274596e-02, inlier_rmse=6.898283e-04, and correspondence_set size of 83
    # Access transformation to get result.
    source_num_points = len(source_down.points)
    target_num_points = len(target_down.points)
    print(target_num_points) #共108點, 成功的是有71個點, 即72％的點都有被配對到
    print(target_num_points/source_num_points)

    evaluation = o3d.pipelines.registration.evaluate_registration(source_down, target_down, threshold, result_ransac.transformation)
    print("global registration evaluation")
    print(evaluation)
    # print(len(evaluation.correspondence_set))
    print("target_num_points=%d" %target_num_points)
    print("paired_point_num=%d" %len(evaluation.correspondence_set))
    paired_point_ratio=len(evaluation.correspondence_set)/target_num_points #target中有幾趴的點又被配對到, 配對到的點太少就要重算global registration
    # print("corresponding point= %.2f" %(target_num_points/len(evaluation.correspondence_set)))
    print("corresponding point= %.2f" %paired_point_ratio)

    while (paired_point_ratio<=0.60) or (evaluation.inlier_rmse > 0.01):
        # evaluation.inlier_rmse 正常的時候是0.00762, 如果這次算出的global誤差太大, 就重算global
        print("recalculate global registration (target_num_points/len(evaluation.correspondence_set))<0.60) or (evaluation.inlier_rmse > 0.01)")
        result_ransac = execute_global_registration(source_down, target_down,
                                                    source_fpfh, target_fpfh,voxel_size)
        evaluation = o3d.pipelines.registration.evaluate_registration(source_down, target_down, threshold, result_ransac.transformation)
        paired_point_ratio=len(evaluation.correspondence_set)/target_num_points
        print("corresponding point= %.2f" %paired_point_ratio)
        print("global registration evaluation")
        print(evaluation)
        draw_registration_result(source_down, target_down, result_ransac.transformation)


    #[Step2] Local Registration (By ICP method)
    icp_count=3
    result_icp_final=ICP_helper(source, target, result_ransac, icp_count, threshold)
    print("Total "+str(icp_count)+"th time of point-to-point ICP")
    draw_registration_result(source_down, target_down, result_icp_final.transformation)
    

    # threshold=0.02
    # print("Apply point-to-point ICP")
    # #下面是open3d內建的icp方法, 可以得到local下的轉換矩陣
    # reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, result_ransac.transformation,o3d.pipelines.registration.TransformationEstimationPointToPoint())
    # print(reg_p2p.transformation)
    # reg_p2p2 = o3d.pipelines.registration.registration_icp(source, target, threshold, reg_p2p.transformation,o3d.pipelines.registration.TransformationEstimationPointToPoint())
    # print(reg_p2p2.transformation)
    # draw_registration_result(source, target, reg_p2p.transformation)

# count=0
#ICP部份用recursive寫
def ICP_helper(source, target, global_result, count, threshold):
    #中止條件
    
    if count<=0:
        return global_result #在最最最內層, 要先從global trans開始
    count -=1
    
    threshold=0.001#0.02
    #下面是open3d內建的icp方法, 可以得到local下的轉換矩陣
    local_result=ICP_helper(source, target, global_result, count, threshold)
    
    new_local_result = o3d.pipelines.registration.registration_icp(source, target, threshold, local_result.transformation,o3d.pipelines.registration.TransformationEstimationPointToPoint())
    

    print(new_local_result.transformation)
    # draw_registration_result(source, target, new_local_result.transformation)

    return new_local_result

    


#Import Dataset
source_path="/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/ros2_ws2/src/dataset/data_pcd/TomatoPlant_size_modified_only1tomato.ply"
target_path="/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/ros2_ws2/src/dataset/data_pcd/copy_of_filtered_msg.pcd"
num=3

# demo_pcds = o3d.data.DemoICPPointClouds()
# source = o3d.io.read_point_cloud(demo_pcds.paths[0])
# target = o3d.io.read_point_cloud(demo_pcds.paths[1])

source = o3d.io.read_point_cloud(source_path.format(num)) #demo_pcds.paths[0])
target = o3d.io.read_point_cloud(target_path.format(num)) #demo_pcds.paths[1])

ICPoperation(source,target)