import numpy as np
import rospy
import time
import sys
import traceback
import std_msgs
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

INTERV_TIME = 0.05

def generate_hoverline(head=[0,-1,0.2], tail=[0,1,0.2], number=3, hover_time=1, interv_time=INTERV_TIME):
    row_ = round(hover_time/interv_time)
    column_ = 3 * number
    waypoints = np.zeros((row_, column_), dtype="float32")
    for row in range(row_):
        for agent_number in range(number):
            waypoints[row][agent_number*3:(agent_number*3 + 3)] = np.array(head) + agent_number*(np.array(tail) - np.array(head))/(number-1)
    return waypoints

def generate_hovercircle(center=[0,0,0.2], radius=0.5, number=3, start_angle=-45, hover_time=1, interv_time=INTERV_TIME):
    row_ = round(hover_time/interv_time)
    column_ = 3 * number
    waypoints = np.zeros((row_, column_), dtype="float32")
    for row in range(row_):
        for agent_number in range(number):
            angle = (360.0 / number)*agent_number + start_angle
            waypoints[row][agent_number*3 + 0] = center[0] + np.cos((angle/57.29577951308232)) * radius
            waypoints[row][agent_number*3 + 1] = center[1] + np.sin((angle/57.29577951308232)) * radius
            waypoints[row][agent_number*3 + 2] = center[2]
    return waypoints

def generate_circle(center=[0,0,0.2], radius=0.5, end_height=-1, speed=0.5, number=3, start_angle=-135, loop=1, interv_time=INTERV_TIME):
    perimeter = 2.0 * np.pi * radius
    w = 57.29577951308232*speed/radius
    row_ = round(perimeter * loop / (interv_time * speed)) + 1
    column_ = 3 * number
    if(end_height > 0):
        raise_deri = (end_height-center[2])/row_
    else:
        raise_deri = 0
    waypoints = np.zeros((row_, column_), dtype="float32")
    for row in range(row_):
        for agent_number in range(number):
            angle = (360.0 / number)*agent_number + start_angle + w * row * interv_time
            waypoints[row][agent_number*3 + 0] = center[0] + np.cos((angle/57.29577951308232)) * radius
            waypoints[row][agent_number*3 + 1] = center[1] + np.sin((angle/57.29577951308232)) * radius
            waypoints[row][agent_number*3 + 2] = center[2] + raise_deri * row
            # print(row, int(angle), waypoints[row][0], waypoints[row][1])
    return waypoints

def generate_snake(waypoint, headtotail=1, headto=[0,0,0], speed=0.5, number=3, interv_time=INTERV_TIME):
    start = waypoint[-1]
    if headtotail==1:
        end = np.hstack((waypoint[-1][-3:],waypoint[-1][0:-3]))
    else:
        end = np.hstack((np.array(headto),waypoint[-1][0:-3]))
    distance = np.zeros((1,number))
    for agent_number in range(number):
        distance[0][agent_number] = np.sqrt(np.sum(np.square(start[agent_number*3:agent_number*3+3] - end[agent_number*3:agent_number*3+3])))
    if min(distance[0])<0.3:
        print("move distance too small", distance[0])
        print(traceback.format_exception_only())

        sys.exit(1)
    row_ = round(max(distance[0])/(interv_time*speed))
    column_ = 3 * number
    waypoints = np.zeros((row_ + 1, column_), dtype="float32")
    for row in range(row_):
        waypoints[row] = start + row * (end - start)/row_
        waypoints[row] = start + row * (end - start)/row_
        waypoints[row] = start + row * (end - start)/row_
    waypoints[-1] = end
    return waypoints

def generate_transition(waypoint1, waypoint2, speed = 0.5, number=3, interv_time = INTERV_TIME):
    start = waypoint1[-1]
    end = waypoint2[0]
    distance = np.zeros((1,number))
    for agent_number in range(number):
        distance[0][agent_number] = np.sqrt(np.sum(np.square(start[agent_number*3:agent_number*3+3] - end[agent_number*3:agent_number*3+3])))
    row_ = round(max(distance[0])/(interv_time*speed))
    column_ = 3 * number
    waypoints = np.zeros((row_ +1 , column_), dtype="float32")
    for row in range(row_):
        waypoints[row] = start + row * (end - start)/row_
        waypoints[row] = start + row * (end - start)/row_
        waypoints[row] = start + row * (end - start)/row_
    waypoints[-1] = end
    return waypoints


def read_waypoint_data(path):
    data = np.loadtxt(path, dtype=np.float32)
    waypoints = [np.zeros((data.shape[0], 3), dtype="float32") for i in range(int(data.shape[1] / 3))]
    for i in range(data.shape[0]):
        for j in range(data.shape[1]):
            waypoints[int(j / 3)][i, j % 3] = data[i, j]
    return waypoints

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
    else:
        points.color = std_msgs.msg.ColorRGBA(1, 1, 1, 1)
    points.points.append(Point(pos[0], pos[1], pos[2]))
    # print(id, pos)
    pub.publish(points)

if __name__ == "__main__":

    rospy.init_node('traj_generate_node')
#   square
    # waypoint0 = generate_hovercircle(center=[0.3, 0.3, 0.2], radius=0.42, start_angle=-135, hover_time=1)
    # waypoint1 = generate_snake(waypoint0, headtotail=1, speed=0.3, headto=[0,0,0.5])
    # waypoint2 = generate_snake(waypoint1, headtotail=1, speed=0.3, headto=[0,0.5,0.5])
    # waypoint3 = generate_snake(waypoint2, headtotail=1, speed=0.3, headto=[0.5,0.5,0.5])
    # waypoint4 = generate_snake(waypoint3, headtotail=1, speed=0.3, headto=[0.5,0.0,0.5])
    # waypoint5 = generate_hovercircle(center=[0.3, 0.3, 0.2], radius=0.42, start_angle=-135, hover_time=1)

    # waypoints = np.vstack((waypoint0, waypoint1, waypoint2, waypoint3, waypoint4, waypoint5))

#   circle
    # waypoint0 = generate_hovercircle(center=[0.3, 0.3, 0.2], radius=0.42, start_angle=-135, hover_time=1)
    # waypoint1 = generate_circle(center=[0.3, 0.3, 0.2], radius=0.42, end_height=1.0, loop=0.5, speed=0.2)
    # waypoint2 = generate_circle(center=[0.3, 0.3, 1.0], radius=0.42, end_height=0.2, loop=0.5, speed=0.2, start_angle=45)
    # # waypoint1to2 = generate_transition(waypoint1, waypoint2, speed=0.1)
    # waypoint3 = generate_hovercircle(center=[0.3, 0.3, 0.2], start_angle=-135, hover_time=0.2)
    # # waypoint2to3 = generate_transition(waypoint2, waypoint3, speed=0.1)

    # waypoints = np.vstack((waypoint0, waypoint1, waypoint2, waypoint3, waypoint0))

#   青瞳编队方案
    # waypoint0 = generate_hoverline([0, -1, 0.2], [0, 1, 0.2], hover_time=1)
    # waypoint1 = generate_hoverline([0, -1, 0.2], [0, 1, 1.8], hover_time=1)
    # waypoint0to1 = generate_transition(waypoint0, waypoint1, speed=0.4)
    # waypoint2 = generate_hoverline([0.4, -1, 0.2], [-0.4, 1, 1.8], hover_time=0.5)
    # waypoint1to2 = generate_transition(waypoint1, waypoint2, speed=0.4)
    # waypoint3 = generate_hoverline([-0.4, -1, 0.2], [0.4, 1, 1.8], hover_time=0.5)
    # waypoint2to3 = generate_transition(waypoint2, waypoint3, speed=0.4)
    # waypoint4 = generate_hovercircle(center=[0, 0, 1], radius=0.42, start_angle=-135, hover_time=1)
    # waypoint3to4 = generate_transition(waypoint3, waypoint4, speed=0.3)
    # waypoint5 = generate_circle(center=[0, 0, 1], radius=0.42, loop=1, speed=0.25)
    # waypoint6 = generate_hovercircle(center=[0, 0, 1], radius=0.42, start_angle=-135, hover_time=1)
    # waypoint6to0 = generate_transition(waypoint4, waypoint0, speed=0.3)
    # waypoints = np.vstack((waypoint0, waypoint0to1,
    #                        waypoint1, waypoint1to2,
    #                        waypoint2, waypoint2to3,
    #                        waypoint3, waypoint3to4,
    #                        waypoint4, 
    #                        waypoint5, 
    #                        waypoint6, waypoint6to0,
    #                        waypoint0))
    
#   自行编队方案
    waypoint0 = generate_hoverline([0, -1, 0.2], [0, 1, 0.2], hover_time=1)
    waypoint1 = generate_hoverline([0, -1, 0.5], [0, 1, 0.5], hover_time=1)
    waypoint0to1 = generate_transition(waypoint0, waypoint1, speed=0.4)
    waypoint2 = generate_hoverline([1, -1, 0.5], [1, 1, 0.5], hover_time=1)
    waypoint1to2 = generate_transition(waypoint1, waypoint2, speed=0.4)
    waypoint3 = generate_hoverline([1, -1, 0.2], [1, 1, 0.2], hover_time=1)
    waypoint2to3 = generate_transition(waypoint2, waypoint3, speed=0.4)
    waypoints = np.vstack((waypoint0, waypoint0to1,
                           waypoint1, waypoint1to2,
                           waypoint2, waypoint2to3,
                           waypoint3))

    
    np.savetxt("line.txt", waypoints, fmt="%.4f")

    ctrl_waypoint = read_waypoint_data("line.txt")

    # print(ctrl_waypoint[0].shape)

    pub = rospy.Publisher("traj_show", Marker, queue_size=10)

    for tt in range(ctrl_waypoint[0].shape[0]):
        if rospy.is_shutdown():
            break
        for ii in range(3):
            show_pos_now(pub, [ctrl_waypoint[ii][tt][0], ctrl_waypoint[ii][tt][1], ctrl_waypoint[ii][tt][2]], ii)
        time.sleep(0.05)