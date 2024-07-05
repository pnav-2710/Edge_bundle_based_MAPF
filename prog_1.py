import numpy as np
import math
import pybullet as p
import time
import pybullet_data
import random
import networkx as nx
import matplotlib.pyplot as plt

class node:
    def __init__(self,pos=[0,0,0],orn=[0,0,0,0]):
        self.pos= pos
        self.orn = orn



class edge:
    def _init_(self, vel=[0,0], time=0, pos=[0,0,0],orn=[0,0,0,0]):
        self.vel = vel
        self.time = time
        self.pos = pos
        self.orn=orn
    def getstate(self):
        return {'vel': self.vel, 'time': self.time, 'pos': self.pos,'orn':self.orn}
    def setstate(self, state):
        self.dict.update(state)

G = nx.DiGraph()

try:
    data = np.load('bundleclass10.npy', allow_pickle=True)
except FileNotFoundError:
    print("The file 'bundle.npy' does not exist.")
else:
    bundle = data.tolist()

def euclidean_distance(point1,point2):
    euclidean_dist = np.linalg.norm(np.array(list(point1))-np.array(list(point2)))
    return euclidean_dist

def find_distance_between_edge_goal(goal_position_local):
    print(goal_position_local)
    edge_goal_distance = []
    no_of_edges= 10
    min = bundle[0]
    min_ed=math.inf
    for e in bundle:
        #print(e,e.pos)
        ed = euclidean_distance(e.pos,goal_position_local)
        edge_goal_distance.append(ed)        
        if(ed < min_ed):
            min_ed = ed
            min = e
    
    #print(edge_goal_distance)
    return min

def transform_and_rotate(position,orientation,original_goal):
    x1,y1,z1 = position
    print("local origin:",position)
    x2,y2,z2 = original_goal
    print("goal acc to global origin:",original_goal)
    new_goal = (x2-x1,y2-y1,z2-z1)
    print("translated goal is: ",new_goal)
    print(p.getEulerFromQuaternion(orientation))
    angle= p.getEulerFromQuaternion(orientation)[2]
    print("the angle of orientation is: ",angle)
    x3,y3,z3 = new_goal
    print(x3*math.cos(angle),y3*math.sin(angle),x3*math.sin(angle),y3*math.cos(angle))
    final_goal = ((x3*math.cos(angle)+y3*math.sin(angle)),(-x3*math.sin(angle)+y3*math.cos(angle)),z3)
    print(final_goal)
    return list(final_goal)


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)
planeId = p.loadURDF("plane.urdf",[0,0,0],p.getQuaternionFromEuler([0,0,0]),useFixedBase=True)
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("robot.urdf",startPos, startOrientation,useFixedBase=True,flags=p.URDF_USE_INERTIA_FROM_FILE|p.URDF_USE_MATERIAL_COLORS_FROM_MTL,physicsClientId=0)
# Initialize an empty list to store the path
#load_bundle=np.load('bundleclass.npy',allow_pickle=True) 
print(bundle)
x = node(pos = [0,0,0],orn = list(p.getQuaternionFromEuler([0,0,0])))
G.add_node(x)
actual_goal = [9,9,0]

for i in range(2):
    if(i == 1):
        pos_x = 9
        pos_y = 9
    else:
        pos_x = random.uniform(0,10)
        pos_y = random.uniform(0,10)
    goal_coordinate = [pos_x,pos_y,0]
    print(goal_coordinate)
    minimum_state = x
    for state in G:

        if(euclidean_distance(state.pos,goal_coordinate)<euclidean_distance(minimum_state.pos,goal_coordinate)):
            minimum_state = state
    print("*******")
    print(minimum_state.pos,minimum_state.orn)
    p.resetBasePositionAndOrientation(boxId,minimum_state.pos,minimum_state.orn) 
    print("current pos and orientation after resetting:",p.getBasePositionAndOrientation(boxId))
    currentPos=minimum_state.pos
    currentOrn=minimum_state.orn
    print(currentPos,currentOrn)
    #goal_coordinate = [-3,-2,0]#Final goal
    goal_position_local = transform_and_rotate(minimum_state.pos, minimum_state.orn, goal_coordinate)

    edge_goal_distance = []#array to store edge_goal_distance
    goal_reached = False
    while goal_reached is not True:#goal_n>(0,0,0)):
        print(goal_reached)
        best_edge=find_distance_between_edge_goal(goal_position_local)
        print(best_edge,goal_reached)
        #print("best edge:", best_edge)
        pos=best_edge.pos
        orn=best_edge.orn

        theta = p.getEulerFromQuaternion(currentOrn)[2]


        print(pos,orn,list(p.getEulerFromQuaternion(orn)))
        #simulate the edge
        p.resetBaseVelocity(boxId,[best_edge.vel[0]*math.cos(theta),best_edge.vel[0]*math.sin(theta), 0.0], [0.0,0.0,best_edge.vel[1]])
        print("applied velocities are:",best_edge.vel[0],best_edge.vel[1])
        rand_time = best_edge.time
        print("randomly chosen time",rand_time)
        no_of_steps=rand_time * 240
        print("no of steps", int(no_of_steps))
        current_step = 0
        while(current_step!=int(no_of_steps)):
            p.stepSimulation()
            time.sleep(1./240.)
            currPos,currOrn=p.getBasePositionAndOrientation(boxId)
            current_step=current_step+1
        curPos, curOrn = p.getBasePositionAndOrientation(boxId)
        print(curPos,curOrn)
        currentPos = list(curPos)
        currentOrn = list(curOrn)
    
        #add node to the graph
    

        #trace the path
        if(euclidean_distance(currentPos,goal_coordinate)<0.5):
            goal_reached = True
        print(goal_reached)
    
        goal_position_local=transform_and_rotate(currentPos,currentOrn,goal_coordinate)
        print('*')
        print(pos,orn,currentPos,currentOrn,goal_position_local)
    state_pos,state_orn = p.getBasePositionAndOrientation(boxId)
    x = node(pos=list(state_pos),orn = list(state_orn))
    G.add_node(x)
    G.add_edge(x,minimum_state)

for i in G:
    print(i.pos)

    
    
# minedge=find_distance_between_edge_goal(goal_n)
# print(orn,pos,goal_n)
p.disconnect()
