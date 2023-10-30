#!/usr/bin/python3
# -*- coding: latin-1 -*-
import rospy
# import threading
from geometry_msgs.msg import PoseStamped, TwistStamped
from socket import *
import time
import transforms3d as tfs
import math
import sys

import struct
from crc import Calculator, Crc16

# 数据包类型
DATA_PACK_TYPE  = 0
PIC_PACK_TYPE   = 1
ACK_PACK_TYPE   = 2
CMD_PACK_TYPE   = 3
STR_PACK_TYPE   = 4
# 控制信息类型
REMOTE_CONTROL_CMD=1
COMMANDER_CMD=2
MOTIONCAP_DATA_CMD=3
# 指令类型
CMD_MAINTAIN=0
TAKE_OFF=1
GOTO=2
MOVE=3
LAND=4
RESET=5
CALIBRATE=6

last_send_time = send_time = 0

class Kuadmini:

    def __init__(self, connect_addr = ('192.168.0.160',1000), number = 0, use_tcp = 1):
        self.frame_cnt = 0
        self.cmd_pack_cnt = 0
        self.last_time = time.time()
        self.mc_x, self.mc_y, self.mc_z, self.mc_vel_x, self.mc_vel_y, self.mc_vel_z, self.mc_roll, self.mc_pitch, self.mc_yaw = 0,0,0,0,0,0,0,0,0
        self.last_mc_x, self.last_mc_y, self.last_mc_z, self.mc_x_lpf, self.mc_y_lpf, self.mc_z_lpf, self.dt = 0,0,0,0,0,0,0
        self.use_tcp = use_tcp
        self.addr = connect_addr
        self.mc_send_cnt = 0
        connected = 0
        if(self.use_tcp == 1):
            self.socket = socket(AF_INET, SOCK_STREAM)
            self.socket.connect(self.addr)
            self.socket.setblocking(False)
            connected = 1
            print("connect to Kuadmini", number, " using TCP", sep="")
        else:
            self.socket = socket(AF_INET, SOCK_DGRAM)
            self.socket.setblocking(False)
            for idx in range(100):
                if idx % 10 == 0:
                    self.socket.sendto("Kgroundstation".encode(), self.addr)
                else:
                    time.sleep(0.01)
                    ret = ""
                    try:
                        ret = self.socket.recv(100)
                    except BlockingIOError as e:
                        pass
                    if ret == "Kuadmini".encode():
                        connected = 1
                        print("connect to Kuadmini", number, " using UDP", sep="")
                        break
        if connected == 0:
            print("connection to Kuadmini failed")
            sys.exit(1)
        self.calculator = Calculator(Crc16.CCITT)

        self.vofa_sock = socket(AF_INET, SOCK_DGRAM)
        if number >= 0:
            self.number = number
            self.motioncap_run()
        # th = threading.Thread(target=self.quadrotor_vel_calc_thread)
        # th.start()

    def send_packet(self, pack_type, pack):
        frame_fmt = "<IIHHH{}sI"

        # 定义数据
        frame_head = 0xff7f7f01
        self.frame_cnt += 1
        pack_len = len(pack)
        crc = self.calculator.checksum(pack)
        print("       \r", crc, end="\r")
        frame_tail = 0x3fff3f01
        # 打包数据
        packed_data = struct.pack(frame_fmt.format(pack_len),
                                frame_head, 
                                self.frame_cnt, 
                                pack_type, 
                                pack_len, 
                                crc, 
                                pack, 
                                frame_tail)
        # print(len(packed_data), end="")
        if(self.use_tcp == 1):
            self.socket.sendall(packed_data)
            try:
                self.socket.recv(10000)
            except BlockingIOError as e:
                pass
        else:
            self.socket.sendto(packed_data, self.addr)

    # 发送动捕信息
    def send_motioncap_pack(self, x, y, z, v_x, v_y, v_z, roll, pitch, yaw):
        self.cmd_pack_cnt+=1
        pk = struct.pack('<IIfffffffff', self.cmd_pack_cnt, MOTIONCAP_DATA_CMD, x, y, z, v_x, v_y, v_z, roll, pitch, yaw)
        self.send_packet(CMD_PACK_TYPE, pk)

    # 发送控制指令
    def send_commander_pack(self, cmd, x=0, y=0, z=0):
        self.cmd_pack_cnt+=1
        pk = struct.pack('<IIIfff', self.cmd_pack_cnt, COMMANDER_CMD, cmd, x, y, z)
        self.send_packet(CMD_PACK_TYPE, pk)

    def takeoff(self):
        self.send_commander_pack(TAKE_OFF)

    def goto(self, x, y, z):
        self.send_commander_pack(GOTO, x, y, z)

    def move(self, x, y, z):
        self.send_commander_pack(MOVE, x, y, z)

    def maintain(self):
        self.send_commander_pack(CMD_MAINTAIN)
    
    def land(self):
        self.send_commander_pack(LAND)

    # ROS话题回调函数
    def quadrotor_posi_cb(self, msg):
        # 解析动捕数据
        ouler = tfs.euler.quat2euler([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
        self.mc_roll = 57.295779579 * ouler[2]  # 当前roll角度
        self.mc_pitch = 57.295779579 * ouler[1]  # 当前pitch角度
        self.mc_yaw = 57.295779579 * ouler[0]  # 当前yaw角度
        self.mc_x = (msg.pose.position.x/1000)  # 当前x位置
        self.mc_y = (msg.pose.position.y/1000)  # 当前y位置
        self.mc_z = (msg.pose.position.z/1000)  # 当前z位置
        self.mc_x_lpf +=  0.8 * (self.mc_x - self.mc_x_lpf) # 位置低通滤波
        self.mc_y_lpf +=  0.8 * (self.mc_y - self.mc_y_lpf) # 位置低通滤波
        self.mc_z_lpf +=  0.8 * (self.mc_z - self.mc_z_lpf) # 位置低通滤波
        self.mc_send_cnt += 1
        # global last_send_time, send_time
        # send_time = time.time()
        # print(send_time-last_send_time)
        # last_send_time = send_time
        if(self.mc_send_cnt % 2 == 0):  # 降低发送频率
            self.send_motioncap_pack(self.mc_x, self.mc_y, self.mc_z, self.mc_vel_x, self.mc_vel_y, self.mc_vel_z, self.mc_roll, self.mc_pitch, self.mc_yaw)
            

        # pk = "mc:{0},{1},{2},{3}\n".format(self.mc_y, self.mc_y_lpf, self.mc_vel_y, self.dt).encode("utf-8")
        # socket.sendto(self.vofa_sock, pk, ("127.0.0.1",1347))
    def quadrotor_vel_calc_thread(self):
        while not rospy.is_shutdown():
            self.this_time = time.time()
            self.dt = (self.this_time - self.last_time)
            # print(self.dt * 1000)
            vel_x = (self.mc_x_lpf - self.last_mc_x)/self.dt # 当前x速度
            vel_y = (self.mc_y_lpf - self.last_mc_y)/self.dt # 当前y速度
            vel_z = (self.mc_z_lpf - self.last_mc_z)/self.dt # 当前z速度
            self.mc_vel_x += 0.35 * (vel_x - self.mc_vel_x) # 速度低通滤波
            self.mc_vel_y += 0.35 * (vel_y - self.mc_vel_y) # 速度低通滤波
            self.mc_vel_z += 0.35 * (vel_z - self.mc_vel_z) # 速度低通滤波
            self.last_time = self.this_time
            self.last_mc_x = self.mc_x_lpf
            self.last_mc_y = self.mc_y_lpf
            self.last_mc_z = self.mc_z_lpf
            time.sleep(0.0167)


    def quadrotor_vel_cb(self, msg):
        # 解析动捕数据
        self.mc_vel_x = (msg.twist.linear.x/1000)  # 当前x速度
        self.mc_vel_y = (msg.twist.linear.y/1000)  # 当前y速度
        self.mc_vel_z = (msg.twist.linear.z/1000)  # 当前z速度

    def motioncap_run(self):
        # 订阅话题
        rospy.Subscriber('/vrpn_client_node/MCServer/{}/pose'.format(self.number),PoseStamped,self.quadrotor_posi_cb)
        rospy.Subscriber('/vrpn_client_node/MCServer/{}/velocity'.format(self.number),TwistStamped,self.quadrotor_vel_cb)
        # while not rospy.is_shutdown():
        #     pass
