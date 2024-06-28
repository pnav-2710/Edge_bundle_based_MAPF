import numpy as np
import math
import pybullet as p
import time
import pybullet_data
import random

def euclidean_distance(point1,point2):
    euclidean_dist = np.linalg.norm(np.array(list(point1))-np.array(list(point2)))
    return euclidean_dist

def find_distance_between_edge_goal(locgoal_pos):
    edge_goal_distance = []
    no_of_edges= 10
    min_ed = math.inf
    best_edge_index = 0
    for j in range(no_of_edges):
        ed = euclidean_distance(load_bundle[j][2],locgoal_pos)
        edge_goal_distance.append(ed)    
        if(ed < min_ed):
            min_ed = ed
            best_edge_index = j
    print(edge_goal_distance)
    return best_edge_index


def transform_and_rotate(position,orientation,original_goal_coordinate,original_goal_orientation):
    angle = p.getEulerFromQuaternion(orientation)[2]
    rotMatrix = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle),  np.cos(angle), 0],
        [0,           0,          1]
    ])

    goal_state_wrt_local_frame = np.matmul(rotMatrix, original_goal_coordinate - np.array(position)) 
    goal_state_wrt_local_frame_orientation = original_goal_orientation[2]-orientation[2]
    return goal_state_wrt_local_frame,goal_state_wrt_local_frame_orientation



physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
#p.setRealTimeSimulation(1)
planeId = p.loadURDF("plane.urdf",[0,0,0],p.getQuaternionFromEuler([0,0,0]),useFixedBase=True)
startPos = np.array([0,0,0])
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("robot.urdf",startPos, startOrientation,useFixedBase=True,flags=p.URDF_USE_INERTIA_FROM_FILE|p.URDF_USE_MATERIAL_COLORS_FROM_MTL,physicsClientId=0)
# Initialize an empty list to store the path
load_bundle=np.load('bundle.npy',allow_pickle=True) 
print(load_bundle)

path = []
pos=startPos
orn=startOrientation
goal_coordinate = np.array([7,7,0])#Final goal
goal_orientation = p.getQuaternionFromEuler([0,0,0])
goal_position_local = goal_coordinate
edge_goal_distance = []#array to store edge_goal_distance
goal_reached = False
while goal_reached is not True:#goal_n>(0,0,0)):
    best_edge=find_distance_between_edge_goal(goal_position_local)
    print(best_edge)
    pos=load_bundle[best_edge][2]
    orn=load_bundle[best_edge][3]
    #simulate the edge
    p.resetBaseVelocity(boxId,[load_bundle[best_edge][0][0],0.0, 0.0], [0.0,0.0,load_bundle[best_edge][0][1]])
    rand_time = load_bundle[best_edge][1]
    print("randomly chosen time",rand_time)
    no_of_steps=rand_time * 240
    print("no of steps", int(no_of_steps))
    current_step = 0
    while(current_step!=int(no_of_steps)):
        p.stepSimulation()
        time.sleep(1./240.)
        currPos,currOrn=p.getBasePositionAndOrientation(boxId)
        path.append((currPos, currOrn))
        current_step=current_step+1
    currentPos, currentOrn = p.getBasePositionAndOrientation(boxId)
    #add node to the graph

    #trace the path
    for i in range(len(path)-1):
        pos1, orn1 = path[i]
        pos2, orn2 = path[i+1]
        p.addUserDebugLine(pos1, pos2, [1, 0, 0], 2, 0)
    path=[]

    goal_position_local,goal_orientation_local=transform_and_rotate(currentPos,currentOrn,goal_coordinate,goal_orientation)
    print('***')
    print(pos,orn,currentPos,currentOrn,goal_position_local,goal_orientation_local)
# minedge=find_distance_between_edge_goal(goal_n)
# print(orn,pos,goal_n)
p.disconnect()
