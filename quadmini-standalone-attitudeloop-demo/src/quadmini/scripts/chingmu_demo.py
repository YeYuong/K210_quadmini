# kuadmini钻环 @asdf
import rospy
import numpy as np
import time
import threading
import std_msgs
from geometry_msgs.msg import PoseStamped, TwistStamped
import transforms3d as tfs
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from kuadmini import Kuadmini

# ————————————————使用说明————————————————
# 飞机说明：开机时第一个音符代表IP最后一位（161/2/3...）；开机时有个IMU自检，不起飞可能是初始化失败，放平稳后按白色按键重启就可以了； RGB亮红色表示电量低；
#   飞得不稳定、中途自动降落一般是网络波动，最好确保路由器信号良好，也有可能是电量不足；降落时不要关闭程序，让动捕持续给它发送位置信息
# 前两个环比较大，有时候会挡住动捕，识别不到飞机
# 由于钻环容错较小，建立刚时最好将坐标系手动移到飞机正中心，我是把飞机放到地系中心建系，然后调整飞机坐标系到地系原点
# 本程序没有做避障算法，合理设置环的位置以及起飞位置
# 圆环速度调到最慢，刚好能旋转的状态
# 动捕中建立圆环刚体时，令刚体x轴指向圆环外侧（即旋转半径方向），令刚体y轴指向装置后方（即圆环的法向量方向，朝向电机尾部），最好调整刚体中心至圆环中心
# 所有元素的y轴朝向法向量方向，即飞机要钻过的方向
# 程序使用圆环的位姿以及已知的旋转半径计算旋转中心，由此计算圆环即将到达的水平位置，作为飞机的等候点
# x轴数值大的为右侧，x轴数值小的为左侧，拱门的x轴应大致朝向右侧，拱门的刚体中心调到容易通过的位置
# 参数列表：
SPIN_RADIUS = 0.51 # (m)圆环中心旋转半径（圆环中心到旋转中心，需要准确测量）
START_ANGLE = 15   # (°)圆环旋转到该角度后飞机开始移动（手动设置的提前量，根据钻环效果调节）
DISTENCE_ADJUST = 0.2 # (m)飞机等候点与环的法向距离（不用调整）
# 主要调节以下参数：
RADIUS_ADJUST_RIGHT = -0.04 # (m)右侧飞机在等候点的水平半径偏移量（负值为靠近旋转中心，根据钻环效果调节）
RADIUS_ADJUST_LEFT = 0.0 # (m)左侧飞机在等候点的水平半径偏移量（负值为靠近旋转中心，根据钻环效果调节）
DESCEND_HEIGHT = 0.1 # (m)飞机在钻环时下降高度（根据钻环效果调节）
ASCEND_HEIGHT = 0.09 # (m)飞机在钻环时上升高度（根据钻环效果调节）

# 动捕刚体编号设置
# kuadmini
R_kuadmini_number = 0
L_kuadmini_number = 8
# step1 still big ring
R_bigring_number = 5  # 大圆环动捕刚体编号
L_bigring_number = 6
# step2 rotating ring
ring0_number = 2  # 圆环动捕刚体编号（无先后顺序）
ring1_number = 3
ring2_number = 4
# step2 arch
arch_number = 7  # 拱门动捕刚体编号

# 变量初始化
R_bigring_mc_pose = [0,0,0]
R_bigring_mc_mat = np.zeros((3,3))
L_bigring_mc_pose = [0,0,0]
L_bigring_mc_mat = np.zeros((3,3))
ring_mc_pose = [[0,0,0], [0,0,0], [0,0,0]]
ring_theta = [0, 120, -120]
ring_last_theta = [0, 120, -120]
ring_mc_mat = [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))]
R_ring_id = 1 # 右侧飞机要钻的圆环序号
L_ring_id = 1 # 左侧飞机要钻的圆环序号
arch_mc_pose = [0,0,0]
arch_mc_mat = np.zeros((3,3))

# 场景元素ROS话题回调函数
def R_bigring_posi_cb(msg):
    global R_bigring_mc_pose, R_bigring_mc_mat
    R_bigring_mc_mat = tfs.quaternions.quat2mat([-msg.pose.orientation.w, msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
    R_bigring_mc_pose[0] = (msg.pose.position.x/1000)  # 当前x位置
    R_bigring_mc_pose[1] = (msg.pose.position.y/1000)  # 当前y位置
    R_bigring_mc_pose[2] = (msg.pose.position.z/1000)  # 当前z位置
def L_bigring_posi_cb(msg):
    global L_bigring_mc_pose, L_bigring_mc_mat
    L_bigring_mc_mat = tfs.quaternions.quat2mat([-msg.pose.orientation.w, msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
    L_bigring_mc_pose[0] = (msg.pose.position.x/1000)  # 当前x位置
    L_bigring_mc_pose[1] = (msg.pose.position.y/1000)  # 当前y位置
    L_bigring_mc_pose[2] = (msg.pose.position.z/1000)  # 当前z位置

def ring0_posi_cb(msg):
    global ring_mc_pose, ring_mc_mat, ring_theta, R_ring_id, L_ring_id
    ring_mc_mat[0] = tfs.quaternions.quat2mat([-msg.pose.orientation.w, msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
    ring_theta[0] = (1 if ring_mc_mat[0][0][2]>=0 else -1) * calc_vector_angle(ring_mc_mat[0][0], (np.cross(ring_mc_mat[0][1].tolist(), [0,0,1])))
    if(ring_theta[0] <= 90 and ring_last_theta[0] > 90):
        R_ring_id = 0
    elif(ring_theta[0] <= -90 and ring_last_theta[0] > -90):
        L_ring_id = 0
    ring_last_theta[0] = ring_theta[0]
    ring_mc_pose[0][0] = (msg.pose.position.x/1000)  # 当前x位置
    ring_mc_pose[0][1] = (msg.pose.position.y/1000)  # 当前y位置
    ring_mc_pose[0][2] = (msg.pose.position.z/1000)  # 当前z位置
def ring1_posi_cb(msg):
    global ring_mc_pose, ring_mc_mat, ring_theta, R_ring_id, L_ring_id
    ring_mc_mat[1] = tfs.quaternions.quat2mat([-msg.pose.orientation.w, msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
    ring_theta[1] = (1 if ring_mc_mat[1][0][2]>=0 else -1) * calc_vector_angle(ring_mc_mat[1][0], (np.cross(ring_mc_mat[1][1].tolist(), [0,0,1])))
    if(ring_theta[1] <= 90 and ring_last_theta[1] > 90):
        R_ring_id = 1
    elif(ring_theta[1] <= -90 and ring_last_theta[1] > -90):
        L_ring_id = 1
    ring_last_theta[1] = ring_theta[1]
    ring_mc_pose[1][0] = (msg.pose.position.x/1000)  # 当前x位置
    ring_mc_pose[1][1] = (msg.pose.position.y/1000)  # 当前y位置
    ring_mc_pose[1][2] = (msg.pose.position.z/1000)  # 当前z位置
def ring2_posi_cb(msg):
    global ring_mc_pose, ring_mc_mat, ring_theta, R_ring_id, L_ring_id
    ring_mc_mat[2] = tfs.quaternions.quat2mat([-msg.pose.orientation.w, msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
    ring_theta[2] = (1 if ring_mc_mat[2][0][2]>=0 else -1) * calc_vector_angle(ring_mc_mat[2][0], (np.cross(ring_mc_mat[2][1].tolist(), [0,0,1])))
    if(ring_theta[2] <= 90 and ring_last_theta[2] > 90):
        R_ring_id = 2
    elif(ring_theta[2] <= -90 and ring_last_theta[2] > -90):
        L_ring_id = 2
    ring_last_theta[2] = ring_theta[2]
    ring_mc_pose[2][0] = (msg.pose.position.x/1000)  # 当前x位置
    ring_mc_pose[2][1] = (msg.pose.position.y/1000)  # 当前y位置
    ring_mc_pose[2][2] = (msg.pose.position.z/1000)  # 当前z位置

def arch_posi_cb(msg):
    global arch_mc_pose, arch_mc_mat
    arch_mc_mat = tfs.quaternions.quat2mat([-msg.pose.orientation.w, msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
    arch_mc_pose[0] = (msg.pose.position.x/1000)  # 当前x位置
    arch_mc_pose[1] = (msg.pose.position.y/1000)  # 当前y位置
    arch_mc_pose[2] = (msg.pose.position.z/1000)  # 当前z位置

# 可以用rviz显示计算的结果（圆环旋转状态，等待点，飞行目标方向）打开rviz按话题添加traj_show即可
Color_Red = std_msgs.msg.ColorRGBA(1, 0, 0, 1)
Color_Green = std_msgs.msg.ColorRGBA(0, 1, 0, 1)
Color_Blue = std_msgs.msg.ColorRGBA(0, 0, 1, 1)
Color_Purple = std_msgs.msg.ColorRGBA(1, 0, 1, 1)
Color_Orange = std_msgs.msg.ColorRGBA(1, 0.5, 0, 1)
Color_Yellow = std_msgs.msg.ColorRGBA(1, 1, 0, 1)
Color_White = std_msgs.msg.ColorRGBA(1, 1, 1, 1)
def show_pos_now(pub, pos, id=0, color=std_msgs.msg.ColorRGBA(1, 1, 1, 0.5)):
    points = Marker()
    points.header.frame_id = 'map'
    points.header.stamp = rospy.Time.now()
    points.ns = 'points_and_lines'
    points.pose.orientation.w = 1.0
    points.pose.orientation.x = 0.0
    points.pose.orientation.y = 0.0
    points.pose.orientation.z = 0.0
    points.action = Marker.ADD
    points.id = id
    points.type = Marker.SPHERE_LIST
    points.scale.x = 0.1
    points.scale.y = 0.1
    points.scale.z = 0.1
    points.color = color
    points.points.append(Point(pos[0], pos[1], pos[2]))
    # print(id, pos)
    pub.publish(points)
def show_arrow_now(pub, start, end, id=0, color=std_msgs.msg.ColorRGBA(1, 1, 1, 0.5)):
    points = Marker()
    points.header.frame_id = 'map'
    points.header.stamp = rospy.Time.now()
    points.ns = 'points_and_lines'
    points.pose.orientation.w = 1.0
    points.pose.orientation.x = 0.0
    points.pose.orientation.y = 0.0
    points.pose.orientation.z = 0.0
    points.action = Marker.ADD
    points.id = id
    points.type = Marker.ARROW
    points.scale.x = 0.04
    points.scale.y = 0.08
    points.scale.z = 0.1
    points.color = color
    points.points.append(Point(start[0], start[1], start[2]))
    points.points.append(Point(end[0], end[1], end[2]))
    # print(id, pos)
    pub.publish(points)

# 计算钻环等候点
def calc_wait_point(pose, mat, radius, right1left2frontPbackM):
    center = np.add(pose, (-radius*mat[0]).tolist()).tolist()   # 计算圆心
    RADIUS_ADJUST = RADIUS_ADJUST_RIGHT if abs(right1left2frontPbackM)==1 else RADIUS_ADJUST_LEFT
    horizontal_radius = (np.cross(mat[1].tolist(), [0,0,1 if abs(right1left2frontPbackM)==1 else -1]) * (radius+RADIUS_ADJUST)).tolist()  # 水平半径向量
    horizontal_point = np.add(center, horizontal_radius)    # 圆周上的水平点
    waitpoint = np.add(horizontal_point,((-1 if right1left2frontPbackM > 0 else 1.5) *DISTENCE_ADJUST*mat[1]).tolist()) # 飞机等候点
    show_arrow_now(pub, center, pose, 8 + right1left2frontPbackM*3)
    show_arrow_now(pub, center, horizontal_point, 9 + right1left2frontPbackM*3)
    show_pos_now(pub, waitpoint, 10 + right1left2frontPbackM*3)
    return waitpoint
# 向目标点移动，但限制给定目标点与当前位置的距离，防止飞行速度太快不稳定
def goto_limit(position, destine, distence_limit):
    d_x = destine[0]-position[0]
    d_y = destine[1]-position[1]
    d_z = destine[2]-position[2]
    length = np.linalg.norm([d_x, d_y, d_z])
    limit_k = 1.0
    if length > distence_limit:
        limit_k = distence_limit/length
    dest_x = position[0] + d_x*limit_k
    dest_y = position[1] + d_y*limit_k
    dest_z = position[2] + d_z*limit_k
    return dest_x, dest_y, dest_z
# 计算两个三维向量的夹角
def calc_vector_angle(vec1, vec2):
    vec_norm = np.linalg.norm(vec1) * np.linalg.norm(vec2)
    cos_ = np.dot(vec1, vec2)/vec_norm
    sin_ = np.linalg.norm(np.cross(vec1, vec2))/vec_norm
    return 57.295779579 * np.arctan2(sin_, cos_)

# 右侧飞机过大环
def right_bigring_task(kuad):
    PROC_TAKEOFF = 0
    PROC_GOTO_WAITPOINT = 1
    PROC_PREPARING = 2
    PROC_GOTHROUGH = 3
    PROC_HOVER = 4
    PROC_FINISHED = 5
    if not hasattr(right_bigring_task, "proc"):
        right_bigring_task.proc = PROC_TAKEOFF
    if not hasattr(right_bigring_task, "cnt"):
        right_bigring_task.cnt = 0
    # 起飞
    if right_bigring_task.proc == PROC_TAKEOFF:
        if right_bigring_task.cnt == 0:
            print("R:起飞     ")
        if right_bigring_task.cnt < 10:
            kuad.takeoff()
            right_bigring_task.cnt+=1
        else:
            right_bigring_task.proc = PROC_GOTO_WAITPOINT
            right_bigring_task.cnt = 0
    # 计算等候点，飞到等候点
    elif(right_bigring_task.proc == PROC_GOTO_WAITPOINT):
        if right_bigring_task.cnt == 0:
            print("R:进入等候区")
        right_bigring_task.cnt+=1
        wait_point = np.add(R_bigring_mc_pose,(-1*DISTENCE_ADJUST*R_bigring_mc_mat[1]).tolist()) # 飞机等候点
        dest_x, dest_y, dest_z = goto_limit([kuad.mc_x, kuad.mc_y, kuad.mc_z], wait_point, 0.2)
        show_pos_now(pub, [kuad.mc_x, kuad.mc_y, kuad.mc_z], 0, Color_Green)
        show_arrow_now(pub, [kuad.mc_x, kuad.mc_y, kuad.mc_z], [dest_x, dest_y, dest_z], 4)
        kuad.goto(dest_x, dest_y, dest_z)
        diff = np.linalg.norm([kuad.mc_x-wait_point[0], kuad.mc_y-wait_point[1], kuad.mc_z-wait_point[2]])
        if(diff < 0.05): # 距离目标点小于5cm则认为到达
            right_bigring_task.proc = PROC_PREPARING
            right_bigring_task.cnt = 0
    # 悬停一秒，准备钻环
    elif(right_bigring_task.proc == PROC_PREPARING):
        if right_bigring_task.cnt < 20:
            right_bigring_task.cnt+=1
            wait_point = np.add(R_bigring_mc_pose,(-1*DISTENCE_ADJUST*R_bigring_mc_mat[1]).tolist())
            kuad.goto(wait_point[0], wait_point[1], wait_point[2])
        else:
            print("R:钻!    ")
            right_bigring_task.proc = PROC_GOTHROUGH
            right_bigring_task.cnt = 0
    # 给飞机一个环y轴方向的目标点
    elif(right_bigring_task.proc == PROC_GOTHROUGH):
        if right_bigring_task.cnt < 30: # 钻环最多1.5秒，防止撞环后失控
            right_bigring_task.cnt+=1
            wait_point = np.add(R_bigring_mc_pose,(-1*DISTENCE_ADJUST*R_bigring_mc_mat[1]).tolist())
            dest_point = [wait_point[0] + R_bigring_mc_mat[1][0]*0.5,
                        wait_point[1] + R_bigring_mc_mat[1][1]*0.5,
                        wait_point[2] + R_bigring_mc_mat[1][2]*0.5 ]
            dest_x, dest_y, dest_z = goto_limit([kuad.mc_x, kuad.mc_y, kuad.mc_z], dest_point, 0.4)
            show_arrow_now(pub, wait_point, dest_point, 1, Color_Blue)
            kuad.goto(dest_x, dest_y, dest_z)
            diff = np.linalg.norm([kuad.mc_x-dest_point[0], kuad.mc_y-dest_point[1], kuad.mc_z-dest_point[2]])
            if(diff < 0.05):
                right_bigring_task.proc = PROC_HOVER
                right_bigring_task.cnt = 0
            # 检测是否撞环
            if abs(kuad.mc_yaw) > 30:
                print("R:撞环！")
                kuad.land()
                print("R:降落   落地前不要关闭程序！")
                right_bigring_task.proc = PROC_FINISHED
                right_bigring_task.cnt = 0
        else:
            right_bigring_task.proc = PROC_HOVER
            right_bigring_task.cnt = 0
    # 悬停等待下一个项目
    elif(right_bigring_task.proc == PROC_HOVER):
        if right_bigring_task.cnt < 60:
            right_bigring_task.cnt+=1
            wait_point = np.add(R_bigring_mc_pose,(1.5*DISTENCE_ADJUST*R_bigring_mc_mat[1]).tolist())
            kuad.goto(wait_point[0], wait_point[1], wait_point[2])
        else:
            kuad.land()
            print("R:降落   落地前不要关闭程序！")
            right_bigring_task.proc = PROC_FINISHED
            right_bigring_task.cnt = 0
    # 故障或超时，任务结束
    elif(right_bigring_task.proc == PROC_FINISHED):
        kuad.land()
        right_bigring_task.cnt = 0
    return right_bigring_task.proc
# 左侧飞机过大环
def left_bigring_task(kuad):
    PROC_TAKEOFF = 0
    PROC_GOTO_WAITPOINT = 1
    PROC_PREPARING = 2
    PROC_GOTHROUGH = 3
    PROC_HOVER = 4
    PROC_FINISHED = 5
    if not hasattr(left_bigring_task, "proc"):
        left_bigring_task.proc = PROC_TAKEOFF
    if not hasattr(left_bigring_task, "cnt"):
        left_bigring_task.cnt = 0
    # 起飞
    if left_bigring_task.proc == PROC_TAKEOFF:
        if left_bigring_task.cnt == 0:
            print("L:起飞     ")
        if left_bigring_task.cnt < 10:
            kuad.takeoff()
            left_bigring_task.cnt+=1
        else:
            left_bigring_task.proc = PROC_GOTO_WAITPOINT
            left_bigring_task.cnt = 0
    # 计算等候点，飞到等候点
    elif(left_bigring_task.proc == PROC_GOTO_WAITPOINT):
        if left_bigring_task.cnt == 0:
            print("L:进入等候区")
        left_bigring_task.cnt+=1
        wait_point = np.add(L_bigring_mc_pose,(-1*DISTENCE_ADJUST*L_bigring_mc_mat[1]).tolist()) # 飞机等候点
        dest_x, dest_y, dest_z = goto_limit([kuad.mc_x, kuad.mc_y, kuad.mc_z], wait_point, 0.2)
        show_pos_now(pub, [kuad.mc_x, kuad.mc_y, kuad.mc_z], 0, Color_Green)
        show_arrow_now(pub, [kuad.mc_x, kuad.mc_y, kuad.mc_z], [dest_x, dest_y, dest_z], 4)
        kuad.goto(dest_x, dest_y, dest_z)
        diff = np.linalg.norm([kuad.mc_x-wait_point[0], kuad.mc_y-wait_point[1], kuad.mc_z-wait_point[2]])
        if(diff < 0.05): # 距离目标点小于5cm则认为到达
            left_bigring_task.proc = PROC_PREPARING
            left_bigring_task.cnt = 0
    # 悬停一秒，准备钻环
    elif(left_bigring_task.proc == PROC_PREPARING):
        if left_bigring_task.cnt < 20:
            left_bigring_task.cnt+=1
            wait_point = np.add(L_bigring_mc_pose,(-1*DISTENCE_ADJUST*L_bigring_mc_mat[1]).tolist()) # 飞机等候点
            kuad.goto(wait_point[0], wait_point[1], wait_point[2])
        else:
            print("L:钻!    ")
            left_bigring_task.proc = PROC_GOTHROUGH
            left_bigring_task.cnt = 0
    # 给飞机一个环y轴方向的目标点
    elif(left_bigring_task.proc == PROC_GOTHROUGH):
        if left_bigring_task.cnt < 30: # 钻环最多1.5秒，防止撞环后失控
            left_bigring_task.cnt+=1
            wait_point = np.add(L_bigring_mc_pose,(-1*DISTENCE_ADJUST*L_bigring_mc_mat[1]).tolist())
            dest_point = [wait_point[0] + L_bigring_mc_mat[1][0]*0.5,
                        wait_point[1] + L_bigring_mc_mat[1][1]*0.5,
                        wait_point[2] + L_bigring_mc_mat[1][2]*0.5]
            dest_x, dest_y, dest_z = goto_limit([kuad.mc_x, kuad.mc_y, kuad.mc_z], dest_point, 0.4)
            show_arrow_now(pub, wait_point, dest_point, 1, Color_Blue)
            kuad.goto(dest_x, dest_y, dest_z)
            diff = np.linalg.norm([kuad.mc_x-dest_point[0], kuad.mc_y-dest_point[1], kuad.mc_z-dest_point[2]])
            if(diff < 0.05):
                left_bigring_task.proc = PROC_HOVER
                left_bigring_task.cnt = 0
            # 检测是否撞环
            if abs(kuad.mc_yaw) > 30:
                print("L:撞环！")
                kuad.land()
                print("L:降落   落地前不要关闭程序！")
                left_bigring_task.proc = PROC_FINISHED
                left_bigring_task.cnt = 0
        else:
            left_bigring_task.proc = PROC_HOVER
            left_bigring_task.cnt = 0
    # 悬停等待下一个项目
    elif(left_bigring_task.proc == PROC_HOVER):
        if left_bigring_task.cnt < 60:
            left_bigring_task.cnt+=1
            wait_point = np.add(L_bigring_mc_pose,(1.5*DISTENCE_ADJUST*L_bigring_mc_mat[1]).tolist())
            kuad.goto(wait_point[0], wait_point[1], wait_point[2])
        else:
            kuad.land()
            print("L:降落   落地前不要关闭程序！")
            left_bigring_task.proc = PROC_FINISHED
            left_bigring_task.cnt = 0
    # 故障或超时，任务结束
    elif(left_bigring_task.proc == PROC_FINISHED):
        kuad.land()
        left_bigring_task.cnt = 0
    return left_bigring_task.proc
# 右侧飞机钻环
def right_spinring_task(kuad):
    PROC_TAKEOFF = 0
    PROC_GOTO_WAITPOINT = 1
    PROC_PREPARING = 2
    PROC_GOTHROUGH = 3
    PROC_HOVER = 4
    PROC_FINISHED = 5
    if not hasattr(right_spinring_task, "proc"):
        right_spinring_task.proc = PROC_TAKEOFF
    if not hasattr(right_spinring_task, "cnt"):
        right_spinring_task.cnt = 0
    if not hasattr(right_spinring_task, "keep_waiting"):
        right_spinring_task.keep_waiting = 0
    # 起飞
    if right_spinring_task.proc == PROC_TAKEOFF:
        if right_spinring_task.cnt == 0:
            print("R:起飞     ")
        if right_spinring_task.cnt < 10:
            kuad.takeoff()
            right_spinring_task.cnt+=1
        else:
            right_spinring_task.proc = PROC_GOTO_WAITPOINT
            right_spinring_task.cnt = 0
    # 计算等候点，飞到等候点
    elif(right_spinring_task.proc == PROC_GOTO_WAITPOINT):
        if right_spinring_task.cnt == 0:
            print("R:进入等候区")
        right_spinring_task.cnt+=1
        wait_point = calc_wait_point(ring_mc_pose[R_ring_id], ring_mc_mat[R_ring_id], SPIN_RADIUS, 1)
        dest_x, dest_y, dest_z = goto_limit([kuad.mc_x, kuad.mc_y, kuad.mc_z], wait_point, 0.2)
        show_pos_now(pub, [kuad.mc_x, kuad.mc_y, kuad.mc_z], 0, Color_Green)
        show_arrow_now(pub, [kuad.mc_x, kuad.mc_y, kuad.mc_z], [dest_x, dest_y, dest_z], 4)
        kuad.goto(dest_x, dest_y, dest_z)
        diff = np.linalg.norm([kuad.mc_x-wait_point[0], kuad.mc_y-wait_point[1], kuad.mc_z-wait_point[2]])
        if(diff < 0.05): # 距离目标点小于5cm则认为到达
            right_spinring_task.proc = PROC_PREPARING
            right_spinring_task.cnt = 0
    # 环y轴角度处于30°到15°之间时飞机在等候点保持稳定悬停，则准备钻环
    elif(right_spinring_task.proc == PROC_PREPARING):
        wait_point = calc_wait_point(ring_mc_pose[R_ring_id], ring_mc_mat[R_ring_id], SPIN_RADIUS, 1)
        kuad.goto(wait_point[0], wait_point[1], wait_point[2])
        theta = ring_theta[R_ring_id]
        if(theta > START_ANGLE and theta <= 30): # 等待圆环到达15°位置，期间飞机要保持稳定悬停
            diff = np.linalg.norm([kuad.mc_x-wait_point[0], kuad.mc_y-wait_point[1], kuad.mc_z-wait_point[2]])
            if(diff < 0.15): # 根据30°到15°之间悬停状态积累分数
                right_spinring_task.keep_waiting += 1 if right_spinring_task.keep_waiting >= 0 else 0
            else: # 如果飞行不稳定则继续等待
                right_spinring_task.keep_waiting = -1
                print("R:不稳定")
        elif(right_spinring_task.keep_waiting > 8 and theta < START_ANGLE): # 到指定角度开始移动钻圈
            print("R:钻!    ")
            right_spinring_task.keep_waiting=0
            right_spinring_task.proc = PROC_GOTHROUGH
    # 环旋转到指定角度时给飞机一个环y轴方向的目标点
    elif(right_spinring_task.proc == PROC_GOTHROUGH):
        if right_spinring_task.cnt < 30: # 钻环最多1.5秒，防止撞环后失控
            right_spinring_task.cnt+=1
            wait_point = calc_wait_point(ring_mc_pose[R_ring_id], ring_mc_mat[R_ring_id], SPIN_RADIUS, 1)
            # kuad.goto(wait_point[0], wait_point[1], wait_point[2])
            dest_point = [wait_point[0] + ring_mc_mat[R_ring_id][1][0]*0.5,
                        wait_point[1] + ring_mc_mat[R_ring_id][1][1]*0.5,
                        wait_point[2] + ring_mc_mat[R_ring_id][1][2]*0.5 - DESCEND_HEIGHT] # 顺势下降一点高度
            dest_x, dest_y, dest_z = goto_limit([kuad.mc_x, kuad.mc_y, kuad.mc_z], dest_point, 0.4)
            show_arrow_now(pub, wait_point, dest_point, 1, Color_Blue)
            kuad.goto(dest_x, dest_y, dest_z)
            diff = np.linalg.norm([kuad.mc_x-dest_point[0], kuad.mc_y-dest_point[1], kuad.mc_z-dest_point[2]])
            if(diff < 0.05):
                right_spinring_task.proc = PROC_HOVER
                right_spinring_task.cnt = 0
            # 检测是否撞环
            if abs(kuad.mc_yaw) > 30:
                print("R:撞环！")
                kuad.land()
                print("R:降落   落地前不要关闭程序！")
                right_spinring_task.proc = PROC_FINISHED
                right_spinring_task.cnt = 0
        else:
            right_spinring_task.proc = PROC_HOVER
            right_spinring_task.cnt = 0
    # 悬停一段时间
    elif(right_spinring_task.proc == PROC_HOVER):
        if right_spinring_task.cnt < 80:
            right_spinring_task.cnt+=1
            wait_point = calc_wait_point(ring_mc_pose[R_ring_id], ring_mc_mat[R_ring_id], SPIN_RADIUS, -1)
            kuad.goto(wait_point[0], wait_point[1], wait_point[2]-DESCEND_HEIGHT)
        else:
            kuad.land()
            print("R:降落   落地前不要关闭程序！")
            right_spinring_task.proc = PROC_FINISHED
            right_spinring_task.cnt = 0
    # 完成穿越
    elif(right_spinring_task.proc == PROC_FINISHED):
        kuad.land()
        right_spinring_task.cnt = 0
    return right_spinring_task.proc
# 左侧飞机钻环
def left_spinring_task(kuad):
    PROC_TAKEOFF = 0
    PROC_GOTO_WAITPOINT = 1
    PROC_PREPARING = 2
    PROC_GOTHROUGH = 3
    PROC_HOVER = 4
    PROC_FINISHED = 5
    if not hasattr(left_spinring_task, "proc"):
        left_spinring_task.proc = PROC_TAKEOFF
    if not hasattr(left_spinring_task, "cnt"):
        left_spinring_task.cnt = 0
    if not hasattr(left_spinring_task, "keep_waiting"):
        left_spinring_task.keep_waiting = 0
    # 起飞
    if left_spinring_task.proc == PROC_TAKEOFF:
        if left_spinring_task.cnt == 0:
            print("L:起飞     ")
        if left_spinring_task.cnt < 10:
            kuad.takeoff()
            left_spinring_task.cnt+=1
        else:
            left_spinring_task.proc = PROC_GOTO_WAITPOINT
            left_spinring_task.cnt = 0
    # 计算等候点，飞到等候点
    elif(left_spinring_task.proc == PROC_GOTO_WAITPOINT):
        if left_spinring_task.cnt == 0:
            print("L:进入等候区")
        left_spinring_task.cnt+=1
        wait_point = calc_wait_point(ring_mc_pose[L_ring_id], ring_mc_mat[L_ring_id], SPIN_RADIUS, 2)
        dest_x, dest_y, dest_z = goto_limit([kuad.mc_x, kuad.mc_y, kuad.mc_z], wait_point, 0.2)
        show_pos_now(pub, [kuad.mc_x, kuad.mc_y, kuad.mc_z], 0, Color_Green)
        show_arrow_now(pub, [kuad.mc_x, kuad.mc_y, kuad.mc_z], [dest_x, dest_y, dest_z], 4)
        kuad.goto(dest_x, dest_y, dest_z)
        diff = np.linalg.norm([kuad.mc_x-wait_point[0], kuad.mc_y-wait_point[1], kuad.mc_z-wait_point[2]])
        if(diff < 0.05): # 距离目标点小于5cm则认为到达
            left_spinring_task.proc = PROC_PREPARING
            left_spinring_task.cnt = 0
    # 环y轴角度处于30°到15°之间时飞机在等候点保持稳定悬停，则准备钻环
    elif(left_spinring_task.proc == PROC_PREPARING):
        wait_point = calc_wait_point(ring_mc_pose[L_ring_id], ring_mc_mat[L_ring_id], SPIN_RADIUS, 2)
        kuad.goto(wait_point[0], wait_point[1], wait_point[2])
        theta = ring_theta[L_ring_id]
        if(theta > -180 + START_ANGLE and theta <= -180 + 30): # 等待圆环到达15°位置，期间飞机要保持稳定悬停
            diff = np.linalg.norm([kuad.mc_x-wait_point[0], kuad.mc_y-wait_point[1], kuad.mc_z-wait_point[2]])
            if(diff < 0.15): # 根据30°到15°之间悬停状态积累分数
                left_spinring_task.keep_waiting += 1 if left_spinring_task.keep_waiting >= 0 else 0
            else: # 如果飞行不稳定则继续等待
                left_spinring_task.keep_waiting = -1
                print("L:不稳定")
        elif(left_spinring_task.keep_waiting > 8 and theta < -180 + START_ANGLE): # 到指定角度开始移动钻圈
            print("L:钻!    ")
            left_spinring_task.keep_waiting=0
            left_spinring_task.proc = PROC_GOTHROUGH
    # 环旋转到指定角度时给飞机一个环y轴方向的目标点
    elif(left_spinring_task.proc == PROC_GOTHROUGH):
        if left_spinring_task.cnt < 30: # 钻环最多1.5秒，防止撞环后失控
            left_spinring_task.cnt+=1
            wait_point = calc_wait_point(ring_mc_pose[L_ring_id], ring_mc_mat[L_ring_id], SPIN_RADIUS, 2)
            # kuad.goto(wait_point[0], wait_point[1], wait_point[2])
            dest_point = [wait_point[0] + ring_mc_mat[L_ring_id][1][0]*0.5,
                        wait_point[1] + ring_mc_mat[L_ring_id][1][1]*0.5,
                        wait_point[2] + ring_mc_mat[L_ring_id][1][2]*0.5 + ASCEND_HEIGHT] # 顺势上升一点高度
            dest_x, dest_y, dest_z = goto_limit([kuad.mc_x, kuad.mc_y, kuad.mc_z], dest_point, 0.4)
            show_arrow_now(pub, wait_point, dest_point, 1, Color_Blue)
            kuad.goto(dest_x, dest_y, dest_z)
            diff = np.linalg.norm([kuad.mc_x-dest_point[0], kuad.mc_y-dest_point[1], kuad.mc_z-dest_point[2]])
            if(diff < 0.05):
                left_spinring_task.proc = PROC_HOVER
                left_spinring_task.cnt = 0
            # 检测是否撞环
            if abs(kuad.mc_yaw) > 30:
                print("L:撞环！")
                kuad.land()
                print("L:降落   落地前不要关闭程序！")
                left_spinring_task.proc = PROC_FINISHED
                left_spinring_task.cnt = 0
        else:
            left_spinring_task.proc = PROC_HOVER
            left_spinring_task.cnt = 0
    # 悬停一段时间
    elif(left_spinring_task.proc == PROC_HOVER):
        if left_spinring_task.cnt < 80:
            left_spinring_task.cnt+=1
            wait_point = calc_wait_point(ring_mc_pose[L_ring_id], ring_mc_mat[L_ring_id], SPIN_RADIUS, -2)
            kuad.goto(wait_point[0], wait_point[1], wait_point[2] + ASCEND_HEIGHT)
        else:
            kuad.land()
            print("L:降落   落地前不要关闭程序！")
            left_spinring_task.proc = PROC_FINISHED
            left_spinring_task.cnt = 0
    # 完成穿越
    elif(left_spinring_task.proc == PROC_FINISHED):
        kuad.land()
        left_spinring_task.cnt = 0
    return left_spinring_task.proc
# 右侧飞机过拱门
def right_arch_task(kuad):
    PROC_TAKEOFF = 0
    PROC_GOTO_WAITPOINT = 1
    PROC_PREPARING = 2
    PROC_GOTHROUGH = 3
    PROC_HOVER = 4
    PROC_FINISHED = 5
    if not hasattr(right_arch_task, "proc"):
        right_arch_task.proc = PROC_TAKEOFF
    if not hasattr(right_arch_task, "cnt"):
        right_arch_task.cnt = 0
    # 起飞
    if right_arch_task.proc == PROC_TAKEOFF:
        if right_arch_task.cnt == 0:
            print("R:起飞     ")
        if right_arch_task.cnt < 10:
            kuad.takeoff()
            right_arch_task.cnt+=1
        else:
            right_arch_task.proc = PROC_GOTO_WAITPOINT
            right_arch_task.cnt = 0
    # 计算等候点，飞到等候点
    elif(right_arch_task.proc == PROC_GOTO_WAITPOINT):
        if right_arch_task.cnt == 0:
            print("R:进入等候区")
        right_arch_task.cnt+=1
        wait_point = np.add(np.add(arch_mc_pose,(-1*DISTENCE_ADJUST*arch_mc_mat[1]).tolist()).tolist(), (0.25*arch_mc_mat[0]).tolist()) # 飞机等候点
        dest_x, dest_y, dest_z = goto_limit([kuad.mc_x, kuad.mc_y, kuad.mc_z], wait_point, 0.2)
        show_pos_now(pub, [kuad.mc_x, kuad.mc_y, kuad.mc_z], 0, Color_Green)
        show_arrow_now(pub, [kuad.mc_x, kuad.mc_y, kuad.mc_z], [dest_x, dest_y, dest_z], 4)
        kuad.goto(dest_x, dest_y, dest_z)
        diff = np.linalg.norm([kuad.mc_x-wait_point[0], kuad.mc_y-wait_point[1], kuad.mc_z-wait_point[2]])
        if(diff < 0.05): # 距离目标点小于5cm则认为到达
            right_arch_task.proc = PROC_PREPARING
            right_arch_task.cnt = 0
    # 悬停0.5秒，准备钻环
    elif(right_arch_task.proc == PROC_PREPARING):
        if right_arch_task.cnt < 10:
            right_arch_task.cnt+=1
            wait_point = np.add(np.add(arch_mc_pose,(-1*DISTENCE_ADJUST*arch_mc_mat[1]).tolist()).tolist(), (0.25*arch_mc_mat[0]).tolist()) # 飞机等候点
            kuad.goto(wait_point[0], wait_point[1], wait_point[2])
        else:
            print("R:钻!    ")
            right_arch_task.proc = PROC_GOTHROUGH
            right_arch_task.cnt = 0
    # 给飞机一个环y轴方向的目标点
    elif(right_arch_task.proc == PROC_GOTHROUGH):
        if right_arch_task.cnt < 30: # 钻环最多1.5秒，防止撞环后失控
            right_arch_task.cnt+=1
            wait_point = np.add(np.add(arch_mc_pose,(-1*DISTENCE_ADJUST*arch_mc_mat[1]).tolist()).tolist(), (0.25*arch_mc_mat[0]).tolist()) # 飞机等候点
            dest_point = [wait_point[0] + arch_mc_mat[1][0]*0.5,
                        wait_point[1] + arch_mc_mat[1][1]*0.5,
                        wait_point[2] + arch_mc_mat[1][2]*0.5 ]
            dest_x, dest_y, dest_z = goto_limit([kuad.mc_x, kuad.mc_y, kuad.mc_z], dest_point, 0.4)
            show_arrow_now(pub, wait_point, dest_point, 1, Color_Blue)
            kuad.goto(dest_x, dest_y, dest_z)
            diff = np.linalg.norm([kuad.mc_x-dest_point[0], kuad.mc_y-dest_point[1], kuad.mc_z-dest_point[2]])
            if(diff < 0.05):
                right_arch_task.proc = PROC_HOVER
                right_arch_task.cnt = 0
            # 检测是否撞环
            if abs(kuad.mc_yaw) > 30:
                print("R:撞环！")
                kuad.land()
                print("R:降落   落地前不要关闭程序！")
                right_arch_task.proc = PROC_FINISHED
                right_arch_task.cnt = 0
        else:
            right_arch_task.proc = PROC_HOVER
            right_arch_task.cnt = 0
    # 悬停等待下一个项目
    elif(right_arch_task.proc == PROC_HOVER):
        if right_arch_task.cnt < 20:
            right_arch_task.cnt+=1
            wait_point = np.add(np.add(arch_mc_pose,(1.5*DISTENCE_ADJUST*arch_mc_mat[1]).tolist()).tolist(), (0.25*arch_mc_mat[0]).tolist()) # 飞机等候点
            kuad.goto(wait_point[0], wait_point[1], wait_point[2])
        else:
            kuad.land()
            print("R:降落   落地前不要关闭程序！")
            right_arch_task.proc = PROC_FINISHED
            right_arch_task.cnt = 0
    # 故障或超时，任务结束
    elif(right_arch_task.proc == PROC_FINISHED):
        kuad.land()
        right_arch_task.cnt = 0
    return right_arch_task.proc
# 左侧飞机过拱门
def left_arch_task(kuad):
    PROC_TAKEOFF = 0
    PROC_GOTO_WAITPOINT = 1
    PROC_PREPARING = 2
    PROC_GOTHROUGH = 3
    PROC_HOVER = 4
    PROC_FINISHED = 5
    if not hasattr(left_arch_task, "proc"):
        left_arch_task.proc = PROC_TAKEOFF
    if not hasattr(left_arch_task, "cnt"):
        left_arch_task.cnt = 0
    # 起飞
    if left_arch_task.proc == PROC_TAKEOFF:
        if left_arch_task.cnt == 0:
            print("L:起飞     ")
        if left_arch_task.cnt < 10:
            kuad.takeoff()
            left_arch_task.cnt+=1
        else:
            left_arch_task.proc = PROC_GOTO_WAITPOINT
            left_arch_task.cnt = 0
    # 计算等候点，飞到等候点
    elif(left_arch_task.proc == PROC_GOTO_WAITPOINT):
        if left_arch_task.cnt == 0:
            print("L:进入等候区")
        left_arch_task.cnt+=1
        wait_point = np.add(np.add(arch_mc_pose,(-1*DISTENCE_ADJUST*arch_mc_mat[1]).tolist()).tolist(), (-0.25*arch_mc_mat[0]).tolist()) # 飞机等候点
        dest_x, dest_y, dest_z = goto_limit([kuad.mc_x, kuad.mc_y, kuad.mc_z], wait_point, 0.2)
        show_pos_now(pub, [kuad.mc_x, kuad.mc_y, kuad.mc_z], 0, Color_Green)
        show_arrow_now(pub, [kuad.mc_x, kuad.mc_y, kuad.mc_z], [dest_x, dest_y, dest_z], 4)
        kuad.goto(dest_x, dest_y, dest_z)
        diff = np.linalg.norm([kuad.mc_x-wait_point[0], kuad.mc_y-wait_point[1], kuad.mc_z-wait_point[2]])
        if(diff < 0.05): # 距离目标点小于5cm则认为到达
            left_arch_task.proc = PROC_PREPARING
            left_arch_task.cnt = 0
    # 悬停0.5秒，准备钻环
    elif(left_arch_task.proc == PROC_PREPARING):
        if left_arch_task.cnt < 10:
            left_arch_task.cnt+=1
            wait_point = np.add(np.add(arch_mc_pose,(-1*DISTENCE_ADJUST*arch_mc_mat[1]).tolist()).tolist(), (-0.25*arch_mc_mat[0]).tolist()) # 飞机等候点
            kuad.goto(wait_point[0], wait_point[1], wait_point[2])
        else:
            print("L:钻!    ")
            left_arch_task.proc = PROC_GOTHROUGH
            left_arch_task.cnt = 0
    # 给飞机一个环y轴方向的目标点
    elif(left_arch_task.proc == PROC_GOTHROUGH):
        if left_arch_task.cnt < 30: # 钻环最多1.5秒，防止撞环后失控
            left_arch_task.cnt+=1
            wait_point = np.add(np.add(arch_mc_pose,(-1*DISTENCE_ADJUST*arch_mc_mat[1]).tolist()).tolist(), (-0.25*arch_mc_mat[0]).tolist()) # 飞机等候点
            dest_point = [wait_point[0] + arch_mc_mat[1][0]*0.5,
                        wait_point[1] + arch_mc_mat[1][1]*0.5,
                        wait_point[2] + arch_mc_mat[1][2]*0.5]
            dest_x, dest_y, dest_z = goto_limit([kuad.mc_x, kuad.mc_y, kuad.mc_z], dest_point, 0.4)
            show_arrow_now(pub, wait_point, dest_point, 1, Color_Blue)
            kuad.goto(dest_x, dest_y, dest_z)
            diff = np.linalg.norm([kuad.mc_x-dest_point[0], kuad.mc_y-dest_point[1], kuad.mc_z-dest_point[2]])
            if(diff < 0.05):
                left_arch_task.proc = PROC_HOVER
                left_arch_task.cnt = 0
            # 检测是否撞环
            if abs(kuad.mc_yaw) > 30:
                print("L:撞环！")
                kuad.land()
                print("L:降落   落地前不要关闭程序！")
                left_arch_task.proc = PROC_FINISHED
                left_arch_task.cnt = 0
        else:
            left_arch_task.proc = PROC_HOVER
            left_arch_task.cnt = 0
    # 悬停等待下一个项目
    elif(left_arch_task.proc == PROC_HOVER):
        if left_arch_task.cnt < 20:
            left_arch_task.cnt+=1
            wait_point = np.add(np.add(arch_mc_pose,(1.5*DISTENCE_ADJUST*arch_mc_mat[1]).tolist()).tolist(), (-0.25*arch_mc_mat[0]).tolist()) # 飞机等候点
            kuad.goto(wait_point[0], wait_point[1], wait_point[2])
        else:
            kuad.land()
            print("L:降落   落地前不要关闭程序！")
            left_arch_task.proc = PROC_FINISHED
            left_arch_task.cnt = 0
    # 故障或超时，任务结束
    elif(left_arch_task.proc == PROC_FINISHED):
        kuad.land()
        left_arch_task.cnt = 0
    return left_arch_task.proc

if __name__ == "__main__":

    addr_kuadminiR=('192.168.0.161',1000) # 直连地址
    addr_kuadminiL=('192.168.0.162',1000) # 直连地址
    addr_kground=('127.0.0.1',2001) # 通过上位机转发

    rospy.init_node('kuadmini_swarm', anonymous=False, disable_signals=True)

    # 订阅圆环动捕话题
    rospy.Subscriber('/vrpn_client_node/MCServer/{}/pose'.format(R_bigring_number),PoseStamped,R_bigring_posi_cb)
    rospy.Subscriber('/vrpn_client_node/MCServer/{}/pose'.format(L_bigring_number),PoseStamped,L_bigring_posi_cb)
    rospy.Subscriber('/vrpn_client_node/MCServer/{}/pose'.format(ring0_number),PoseStamped,ring0_posi_cb)
    rospy.Subscriber('/vrpn_client_node/MCServer/{}/pose'.format(ring1_number),PoseStamped,ring1_posi_cb)
    rospy.Subscriber('/vrpn_client_node/MCServer/{}/pose'.format(ring2_number),PoseStamped,ring2_posi_cb)
    rospy.Subscriber('/vrpn_client_node/MCServer/{}/pose'.format(arch_number),PoseStamped,arch_posi_cb)


    kuadmini_R = Kuadmini(addr_kuadminiR, number=R_kuadmini_number, use_tcp=1)
    kuadmini_L = Kuadmini(addr_kuadminiL, number=L_kuadmini_number, use_tcp=1)
    # kuadmini_0 = Kuadmini(addr_kground, number=0, use_tcp=1)

    pub = rospy.Publisher("traj_show", Marker, queue_size=10)

    time.sleep(1) # 等待动捕数据

    # 检查场地状态（环是否摆正，飞机位置等）
    if(sum(ring_mc_pose[0])==0):
        print("警告：没有收到指定的动捕话题")
        quit()
    if(np.dot(ring_mc_mat[0][1], ring_mc_mat[0][1,:2].tolist()+[0]) < 0.98 or np.dot(ring_mc_mat[1][1], ring_mc_mat[1][1,:2].tolist()+[0]) < 0.98 or np.dot(ring_mc_mat[2][1], ring_mc_mat[2][1,:2].tolist()+[0]) < 0.98):
        print("警告：请将圆环摆正")
        quit()

    # 钻大环
    while not rospy.is_shutdown():
        left_proc = left_bigring_task(kuadmini_L)
        right_proc = right_bigring_task(kuadmini_R)
        if left_proc == right_proc == 4:
            break
        time.sleep(0.05)
    # 钻旋转环
    while not rospy.is_shutdown():
        left_proc = left_spinring_task(kuadmini_L)
        right_proc = right_spinring_task(kuadmini_R)
        if left_proc == right_proc == 4:
            break
        time.sleep(0.05)
    # 钻拱门
    while not rospy.is_shutdown():
        left_proc = left_arch_task(kuadmini_L)
        right_proc = right_arch_task(kuadmini_R)
        if left_proc == right_proc == 5:
            break
        time.sleep(0.05)

    # 测试显示
    # while not rospy.is_shutdown():
    #     # print(ring_mc_pose[0], ring_mc_mat[0][0])
    #     # show_arrow_now(pub, ring_mc_pose[0], np.add(ring_mc_pose[0], 0.4*ring_mc_mat[0][0]).tolist(), 0)
    #     # show_arrow_now(pub, ring_mc_pose[0], np.add(ring_mc_pose[0], 0.4*ring_mc_mat[0][1]).tolist(), 1)
    #     # show_arrow_now(pub, ring_mc_pose[0], np.add(ring_mc_pose[0], 0.4*ring_mc_mat[0][2]).tolist(), 2)
    #     # calc_wait_point(ring_mc_pose[R_ring_id], ring_mc_mat[R_ring_id], SPIN_RADIUS, 1)
    #     # calc_wait_point(ring_mc_pose[L_ring_id], ring_mc_mat[L_ring_id], SPIN_RADIUS, 2)
    #     calc_wait_point(ring_mc_pose[R_ring_id], ring_mc_mat[R_ring_id], SPIN_RADIUS, 1)
    #     calc_wait_point(ring_mc_pose[L_ring_id], ring_mc_mat[L_ring_id], SPIN_RADIUS, 2)
    #     # print("\r%.2f     "%np.dot(ring_mc_mat[0][1], ring_mc_mat[0][1,:2].tolist()+[0]),end="")
    #     # horizontal_radius = (np.cross(ring_mc_mat[0][1].tolist(), [0,0,1]) * SPIN_RADIUS)  # 水平半径向量
    #     theta = ring_theta[R_ring_id]
    #     print("\r                   %.1f"%(theta),end="    ")
    #     # print("\r                   %.1f, %.1f, %.1f"%(ring1_mc_euler[0],ring1_mc_euler[1],ring1_mc_euler[2]),end="    ")
    #     time.sleep(0.1)

   

    while not rospy.is_shutdown():
        pass

