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
    data = np.load('bundle.npy', allow_pickle=True)
except FileNotFoundError:
    print("The file 'bundle.npy' does not exist.")
else:
    bundle = data.tolist()

def euclidean_distance(point1,point2):
    euclidean_dist = np.linalg.norm(np.array(point1)-np.array(point2))
    return euclidean_dist

def partition(bundle,low,high,goal_position_local):
    pivot=bundle[high]
    i=low-1
    for j in range(low,high):
        if(euclidean_distance(bundle[j].pos,goal_position_local)<euclidean_distance(pivot.pos,goal_position_local)):
            i=i+1
            (bundle[i],bundle[j])=(bundle[j],bundle[i])
    (bundle[i+1],bundle[high])=(bundle[high],bundle[i+1])
    return i+1

def quickSort(bundle,low,high,goal_position_local):
    if(low<high):
        pi=partition(bundle,low,high,goal_position_local)
        quickSort(bundle,low,pi-1,goal_position_local)
        quickSort(bundle,pi+1,high,goal_position_local)

def find_distance_between_edge_goal(goal_position_local):
    print(goal_position_local)
    edge_goal_distance = []
    #no_of_edges= 10
    min = 0
    min_ed=math.inf
    for e in bundle:
        ed = euclidean_distance(e.pos,goal_position_local)
        edge_goal_distance.append(ed)        
        if(ed < min_ed):
            min_ed = ed
            min = e
    size=10
    quickSort(bundle,0,size-1,goal_position_local)
    #print(bundle)
    print(min)
    #print(edge_goal_distance)
    return min

def transform_and_rotate(position,orientation,original_goal):
    x1,y1,z1 = position
    print("local origin:",position,x1,y1,z1)
    x2,y2,z2 = original_goal
    print("goal acc to global origin:",original_goal,x2,y2,z2)
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
#p.setRealTimeSimulation(1)
planeId = p.loadURDF("plane.urdf",[0,0,0],p.getQuaternionFromEuler([0,0,0]),useFixedBase=True)
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("robot.urdf",startPos, startOrientation,useFixedBase=True,flags=p.URDF_USE_INERTIA_FROM_FILE|p.URDF_USE_MATERIAL_COLORS_FROM_MTL,physicsClientId=0)
# Initialize an empty list to store the path
#load_bundle=np.load('bundleclass.npy',allow_pickle=True) 
#print(bundle)
x = node(pos = [0,0,0],orn = list(p.getQuaternionFromEuler([0,0,0])))
G.add_node(x)
actual_goal = [2,2,0]
goal_reached = False

#Obstacle 
obstacleSize = [0.25, 0.25, 0.25]  # Width, Length, Height
obstaclePosition = [1,1,0]  # X, Y, Z position relative to the origin
obstacleOrientation = [0, 0, 0, 1]  # Orientation in Euler angles

obstacleId = p.createCollisionShape(p.GEOM_BOX, halfExtents=obstacleSize)
obstacleVisualShape = p.createVisualShape(p.GEOM_BOX, halfExtents=obstacleSize, rgbaColor=[1, 0, 0, 1])  # Red color
obstacle = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=obstacleId, baseVisualShapeIndex=obstacleVisualShape, basePosition=obstaclePosition, baseOrientation=obstacleOrientation)

# #Collision Detection
# col_detector = p.CollisionDetector()
# col_detector.add_body(boxId)
# col_detector.add_body(obstacle)

#List to store obstacles
obstacle_list=[obstacle]

while goal_reached is not True:#goal_n>(0,0,0)):
    chance = random.uniform(0,100)
    if(0<=int(chance)<90):
        pos_x = random.uniform(0,10)
        pos_y = random.uniform(0,10)
    else:
        pos_x = 2
        pos_y = 2

    goal_coordinate = [pos_x,pos_y,0]
    text=p.addUserDebugText("X", goal_coordinate, [1, 0, 0], textSize=1)

    print(goal_coordinate)
    print(goal_reached)
    minimum_state = x
    for state in G:
        if((euclidean_distance(state.pos,goal_coordinate)) < (euclidean_distance(minimum_state.pos,goal_coordinate))):
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
    
    flag=1
    edgenum=0
    while(True):
        print("-------")
        print(flag)
        if(flag==0):
            break
        else:
            flag=0
            best_edge=bundle[edgenum]
            p.resetBaseVelocity(boxId,[best_edge.vel[0]*math.cos(theta),best_edge.vel[0]*math.sin(theta), 0.0], [0.0,0.0,best_edge.vel[1]])
            print("applied velocities are:",best_edge.vel[0],best_edge.vel[1])
        pathmark=[]
        current_step = 0
        while(current_step!=int(no_of_steps)):
            p.stepSimulation()
            time.sleep(1./240.)
            currPos,currOrn=p.getBasePositionAndOrientation(boxId)
            #joint_states = p.getJointStates(boxId)
            # d = col_detector.compute_distances(joint_states)
            # in_col = col_detector.in_collision(joint_states)
            pathmark.append((currPos, currOrn))
            for i in obstacle_list:
                closest_points = p.getClosestPoints(bodyA=boxId, bodyB=i,distance = 0.1, physicsClientId=physicsClient)
                #print(closest_points)
                if(len(closest_points)!=0):#Collision in the edge
                    edgenum=edgenum+1
                    flag=1

                    print("closest pointsss:",closest_points)
                    break
            
            if flag==1:
                break
            current_step=current_step+1      
    curPos, curOrn = p.getBasePositionAndOrientation(boxId)
    currentPos = list(curPos)
    currentOrn = list(curOrn)
    listmarkerid=[]
    k=0
    for i in range(len(pathmark)-1):
        pos1, orn1 = pathmark[i]
        pos2, orn2 = pathmark[i+1]
        x=p.addUserDebugLine(pos1, pos2, [1, 0, 0], 2, 0)
        listmarkerid.append(x)
        k+=1

    time.sleep(1)
    p.removeUserDebugItem(text)
    k=0
    for i in range(len(pathmark)-1):
        pos1, orn1 = pathmark[i]
        pos2, orn2 = pathmark[i+1]
        p.removeUserDebugItem(listmarkerid[k])
        k+=1
    

    #add node to the graph
    x = node(pos = currentPos , orn = currentOrn)
    G.add_node(x)
    G.add_edge(minimum_state,x)

    #trace the path
    print(euclidean_distance(currentPos,actual_goal))
    if(euclidean_distance(currentPos,actual_goal)<0.5):
        #print(euclidean_distance(currentPos,goal_coordinate))
        goal_reached = True
    
    #goal_position_local=transform_and_rotate(currentPos,currentOrn,goal_coordinate)
    print('*')
    print(pos,orn,currentPos,currentOrn,goal_position_local)
    print("****** ",goal_reached)
# minedge=find_distance_between_edge_goal(goal_n)
# print(orn,pos,goal_n)

for n in G:
    print(n)
print(n)
path = []
path.append(n)
print(path)
while(n.pos!=[0,0,0]):
    incoming_edges_to_C = G.in_edges(n)

    for edge in incoming_edges_to_C:
        print(edge)
        print(edge[0].pos)
        n = edge[0]
        path.append(n)
print(path)

p.disconnect()
