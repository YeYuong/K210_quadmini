import rospy
import numpy as np
import time
import std_msgs
from geometry_msgs.msg import PoseStamped, TwistStamped
import transforms3d as tfs
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from kuadmini import Kuadmini

# 小车动捕刚体编号
car_number = 4
car_mc_x,car_mc_y,car_mc_z = 0,0,0

# 小车ROS话题回调函数
def rikibot_posi_cb(msg):
    # 解析动捕数据
    global car_mc_x,car_mc_y,car_mc_z
    car_mc_x = (msg.pose.position.x/1000)  # 当前x位置
    car_mc_y = (msg.pose.position.y/1000)  # 当前y位置
    car_mc_z = (msg.pose.position.z/1000)  # 当前z位置

def show_pos_now(pub, pos, id=0):
    points = Marker()
    points.header.frame_id = 'world'
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
    if id==0:
        points.color = std_msgs.msg.ColorRGBA(0, 0, 1, 1)
    elif id==1:
        points.color = std_msgs.msg.ColorRGBA(0, 1, 0, 1)
    elif id==2:
        points.color = std_msgs.msg.ColorRGBA(1, 0, 0, 1)
    elif id==3:
        points.color = std_msgs.msg.ColorRGBA(1, 0.5, 0, 1)
    elif id==4:
        points.color = std_msgs.msg.ColorRGBA(0, 0, 1, 0.3)
    elif id==5:
        points.color = std_msgs.msg.ColorRGBA(0, 1, 0, 0.3)
    elif id==6:
        points.color = std_msgs.msg.ColorRGBA(1, 0, 0, 0.3)
    elif id==7:
        points.color = std_msgs.msg.ColorRGBA(1, 0.5, 0, 0.3)
    points.points.append(Point(pos[0], pos[1], pos[2]))
    # print(id, pos)
    pub.publish(points)

if __name__ == "__main__":

    addr_kuadmini0=('192.168.0.160',1000) # 直连地址
    # addr_kuadmini1=('192.168.0.161',1000) # 直连地址
    # addr_kuadmini2=('192.168.0.162',1000) # 直连地址
    # addr_kuadmini3=('192.168.0.163',1000) # 直连地址
    # addr_vofa=('127.0.0.1',1347)
    addr_kground=('127.0.0.1',2001) # 通过上位机转发

    rospy.init_node('kuadmini_swarm', anonymous=False, disable_signals=True)

    # 订阅小车动捕话题
    rospy.Subscriber('/vrpn_client_node/MCServer/{}/pose'.format(car_number),PoseStamped,rikibot_posi_cb)

    kuadmini_0 = Kuadmini(addr_kground, number=5, use_tcp=1)
    # kuadmini_1 = Kuadmini(addr_kuadmini1, number=1, use_tcp=1)
    # kuadmini_2 = Kuadmini(addr_kuadmini2, number=2, use_tcp=1)
    # kuadmini_3 = Kuadmini(addr_kuadmini3, number=3, use_tcp=1)
    # kuadmini_k = Kuadmini(addr_kground, number=2, use_tcp=1)

    pub = rospy.Publisher("traj_show", Marker, queue_size=10)

    time.sleep(1)

    d_x = car_mc_x-kuadmini_0.mc_x
    d_y = car_mc_y-kuadmini_0.mc_y
    d_z = car_mc_z-kuadmini_0.mc_z
    length = np.sqrt(d_x*d_x + d_y*d_y)
    if length > 0.3 or d_z >= 0.0:
        print("把飞机放在车上")
        quit()

    for __ in range(5):
        kuadmini_0.takeoff()
        time.sleep(0.1)
    
    while not rospy.is_shutdown():
        d_x = car_mc_x-kuadmini_0.mc_x
        d_y = car_mc_y-kuadmini_0.mc_y
        d_z = car_mc_z+0.2-kuadmini_0.mc_z
        length = np.sqrt(d_x*d_x + d_y*d_y + d_z*d_z)
        limit_k = 1.0
        if length > 0.3:
            limit_k = 0.3/length
        dest_x = kuadmini_0.mc_x + d_x*limit_k
        dest_y = kuadmini_0.mc_y + d_y*limit_k
        dest_z = kuadmini_0.mc_z + d_z*limit_k
        show_pos_now(pub, [car_mc_x, car_mc_y, car_mc_z], 0)
        show_pos_now(pub, [dest_x, dest_y, dest_z], 1)
        show_pos_now(pub, [kuadmini_0.mc_x, kuadmini_0.mc_y, kuadmini_0.mc_z], 2)
        kuadmini_0.goto(dest_x, dest_y, dest_z)
        time.sleep(0.1)

    # goHover(kuadmini_0)# kuadmini_0, kuadmini_1, kuadmini_2,
    # goSwings((kuadmini_0,))
    # goTrajactory((kuadmini_0, kuadmini_1, kuadmini_2, ), pub, "square.txt")

    while not rospy.is_shutdown():
        pass

