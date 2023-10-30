# 用于测试动捕系统
import rospy
import numpy as np
import time
import std_msgs
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from kuadmini import Kuadmini

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

def goHover(kuad):
    time.sleep(0.2)
    dest_x = kuad.mc_x
    dest_y = kuad.mc_y
    print("\ntake off!")
    kuad.takeoff()
    time.sleep(0.1)
    print("\nhover!")
    for __ in range(30):
        kuad.goto(dest_x,dest_y,0.3)
        time.sleep(0.2)
    print("\nland!")
    kuad.land()
    time.sleep(0.1)

def goHovers(kuads):
    time.sleep(0.2)
    print("\ntake off!")
    for kuad in kuads:
        kuad.takeoff()
    time.sleep(1.5)

    print("\nland!")
    for kuad in kuads:
        kuad.land()
    time.sleep(0.1)

def goSquare(kuad):#单个走正方形
    time.sleep(0.2)
    print("\ntake off!")
    # for theta in range(5):
    kuad.takeoff()
    time.sleep(0.1)
    print("\nraise!")
    sp_x, sp_y = kuad.mc_x, kuad.mc_y
    for theta in range(10):
        kuad.goto(sp_x, sp_y, 0.3)
        time.sleep(0.1)
    print("\nforward!")
    for theta in range(30):
        kuad.goto(sp_x+0.5, sp_y+0.0, 0.5)
        time.sleep(0.1)
    print("\nright!")
    for theta in range(30):
        kuad.goto(sp_x+0.5, sp_y-0.5, 0.5)
        time.sleep(0.1)
    print("\nback!")
    for theta in range(30):
        kuad.goto(sp_x+0.0, sp_y-0.5, 0.5)
        time.sleep(0.1)
    print("\nleft!")
    for theta in range(30):
        kuad.goto(sp_x+0.0, sp_y+0.0, 0.5)
        time.sleep(0.1)
    print("\nland!")
    # for theta in range(5):
    kuad.land()
    time.sleep(0.1)


if __name__ == "__main__":

    addr_kuadmini0=('192.168.0.160',1000) # 直连地址
    addr_kuadmini1=('192.168.0.161',1000) # 直连地址
    addr_kuadmini2=('192.168.0.162',1000) # 直连地址
    addr_kuadmini3=('192.168.0.163',1000) # 直连地址
    addr_vofa=('127.0.0.1',1347)
    addr_kground=('127.0.0.1',2001) # 通过上位机转发
    rospy.init_node('kuadmini_swarm', anonymous=False, disable_signals=True)

    # kuadmini_0 = Kuadmini(addr_kground, number=0, use_tcp=1)
    # kuadmini_1 = Kuadmini(addr_kuadmini1, number=0, use_tcp=1)
    # kuadmini_2 = Kuadmini(addr_kuadmini2, number=2, use_tcp=1)
    # kuadmini_3 = Kuadmini(addr_kuadmini3, number=3, use_tcp=1)
    kuadmini_k = Kuadmini(addr_kground, number=10, use_tcp=1)

    # pub = rospy.Publisher("traj_show", Marker, queue_size=10)

    # goSquare(kuadmini_k)
    # goHover(kuadmini_k)# kuadmini_0, kuadmini_1, kuadmini_2,
    # goSwings((kuadmini_0,))
    # goTrajactory((kuadmini_0, kuadmini_1, kuadmini_2, ), pub, "square.txt")

    while not rospy.is_shutdown():
        time.sleep(1)
        pass

