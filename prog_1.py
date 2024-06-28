import numpy as np
import math
import pybullet as p
import time
import pybullet_data
import random

def euclidean_distance(point1,point2):
    x1, y1, z1 = point1
    x2, y2, z2 = point2
    diff_x = x2 - x1
    diff_y = y2 - y1
    diff_z = z2 - z1
    sum_diff = diff_x*2 + diff_y*2 + diff_z*2
    return math.sqrt(sum_diff)

def find_distance_between_edge_goal(locgoal_pos,):
    edge_goal_distance = []
    no_of_edges= 5
    min = 0
    for j in range(no_of_edges):
        ed = euclidean_distance(load_bundle[j][2],locgoal_pos)
        edge_goal_distance.append(ed)
        if(j==0):
            min_ed = ed
        else:
            if(ed < min_ed):
                min_ed = ed
                min = j
    print(edge_goal_distance)
    return min


def transform_and_rotate(position,orientation,original_goal):
    x1,y1,z1 = position
    x2,y2,z2 = original_goal
    new_goal = (x2-x1,y2-y1,z2-z1)
    
    angle= p.getEulerFromQuaternion(orientation)[2]
    rotation_matrix = [[math.cos(angle), math.sin(angle)],
                      [-math.sin(angle), math.cos(angle)]]
    x3,y3,z3 = new_goal
    final_goal = (x3*math.cos(angle)-y3*math.sin(angle),x3*math.sin(angle)+y3*math.cos(angle),z3)
    return final_goal



physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("robot.urdf",startPos, startOrientation,useFixedBase=True,flags=p.URDF_USE_INERTIA_FROM_FILE|p.URDF_USE_MATERIAL_COLORS_FROM_MTL,physicsClientId=0)
# Initialize an empty list to store the path
load_bundle=np.load('bundle.npy',allow_pickle=True) 
print(load_bundle)
pos=startPos
orn=startOrientation
goal_pos = (7,7,0)#Final goal
goal_n=goal_pos#Altered goal
edge_goal_distance = []#array to store edge_goal_distance

while(goal_n>(0,0,0)):
    minedge=find_distance_between_edge_goal(goal_n)
    pos=load_bundle[minedge][2]
    orn=load_bundle[minedge][3]
    goal_n=transform_and_rotate(pos,orn,goal_n)
    print('***')
    print(orn,pos,goal_n)
# minedge=find_distance_between_edge_goal(goal_n)
# print(orn,pos,goal_n)
p.disconnect()