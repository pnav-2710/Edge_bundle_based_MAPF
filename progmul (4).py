import numpy as np
import math
import pybullet as p
import time
import pybullet_data
import random
import networkx as nx
import matplotlib.pyplot as plt

#node in graph
class node:
    def __init__(self,pos=[0,0,0],orn=[0,0,0,0],vel = [0,0],time = 0):
        self.pos= pos
        self.orn = orn
        self.vel = vel
        self.time = time

#edge in bundle
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

#Graph to store nodes 
G = nx.DiGraph()

try:
    data = np.load('bundle.npy', allow_pickle=True)
except FileNotFoundError:
    print("The file 'bundle.npy' does not exist.")
else:
    bundle = data.tolist()

#Finds euclidean distance between 3 points 
def euclidean_distance(point1,point2):
    euclidean_dist = np.linalg.norm(np.array(point1)-np.array(point2))
    return euclidean_dist

#
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

#returns the best edge which leads the robot closer to its goal_coordinate
def find_distance_between_edge_goal(goal_position_local):
    print(goal_position_local)
    edge_goal_distance = []
    min = 0
    min_ed=math.inf
    for e in bundle:
        ed = euclidean_distance(e.pos,goal_position_local)
        edge_goal_distance.append(ed)        
        if(ed < min_ed):
            min_ed = ed
            min = e
    size=100
    quickSort(bundle,0,size-1,goal_position_local)
    print(min)
    return min

#transforms the actual goal according to the new origin
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

#function to execute the final path after state space exploration
def execute_path():
    print("EXECUTING PATH@@@@@@@@YAYYYYYYY")
    no_of_nodes=[]
    current_node_list=[]
    for i in range(len(pathlist)):
        no_of_nodes.append(len(pathlist[i])-1)
        current_node_list.append(pathlist[i][no_of_nodes[i]])
        print(current_node_list[i].pos)

    next_node_list=[]
    
    
    count=max(no_of_nodes)
    print(count)
    for i in range(count):#for iterating through each edge
        print("sssssssssssssssssssssssssss",i)
        max_randtime=0
        for j in range(len(pathlist)):#for iterating through each path 
            no_of_nodes[j]=no_of_nodes[j]-1
            next_node=pathlist[j][no_of_nodes[j]]#stores next node for each path
            next_node_list.append(next_node)
            if(next_node_list[j].time>max_randtime):#for finding max execution time for each edge
                max_randtime=next_node_list[j].time
    
        no_of_steps=int(max_randtime* 240)
        for j in range(len(listbox)):#checks if edges are exhausted
            if(no_of_nodes[j]<0):
                p.resetBaseVelocity(listbox[j][0],[0.0,0.0, 0.0],[0.0,0.0,0.0])
            else:
                p.resetBasePositionAndOrientation(listbox[j][0],current_node_list[j].pos,current_node_list[j].orn)
                theta= p.getEulerFromQuaternion(current_node_list[j].orn)[2]
                p.resetBaseVelocity(listbox[j][0],[next_node_list[j].vel[0]*math.cos(theta),next_node_list[j].vel[0]*math.sin(theta), 0.0],[0.0,0.0,next_node_list[j].vel[1]])

        current_step=0
        steps_list = []#used to store number of steps in each edge
        for m in range(len(pathlist)):
            steps_list.append(int(next_node_list[m].time * 240))
        while(current_step!=int(no_of_steps)):
            boolrob=[]#boolean array that stores if robot was stopped 
            for h in range(len(listbox)):
                boolrob.append(0)
            #Performs collsion check between robots
            for x in range(len(current_node_list)):
                for y in range(len(listbox)):
                    if(listbox[y][0]!=listbox[x][0]):
                        closest_points=p.getClosestPoints(bodyA=listbox[x][0],bodyB=listbox[y][0],distance=0.1,physicsClientId=physicsClient)
                        if(len(closest_points)!=0):
                            if(y>x):#checks for priority based on index
                                if(boolrob[y]==0):#higher priority robot stops and its current edge is complete 
                                    if(steps_list[x]<=current_step):#
                                        theta= p.getEulerFromQuaternion(p.getBasePositionAndOrientation(listbox[y][0])[1])[2]
                                        p.resetBaseVelocity(listbox[y][0],[next_node_list[y].vel[0]*math.cos(theta),next_node_list[y].vel[0]*math.sin(theta), 0.0],[0.0,0.0,next_node_list[y].vel[1]])
                                    else:
                                        steps_list[y] = steps_list[y] + 1
                                        p.resetBaseVelocity(listbox[y][0],[0.0,0.0, 0.0],[0.0,0.0,0.0])
                                        boolrob[y]=1
                                        no_of_steps=no_of_steps+1
                            else:
                                if(boolrob[x]==0):
                                    if(steps_list[y]<=current_step):
                                        theta= p.getEulerFromQuaternion(p.getBasePositionAndOrientation(listbox[x][0])[1])[2]
                                        p.resetBaseVelocity(listbox[x][0],[next_node_list[x].vel[0]*math.cos(theta),next_node_list[x].vel[0]*math.sin(theta), 0.0],[0.0,0.0,next_node_list[x].vel[1]])
                                    else:
                                        steps_list[x] = steps_list[x] + 1
                                        p.resetBaseVelocity(listbox[x][0],[0.0,0.0, 0.0],[0.0,0.0,0.0])
                                        boolrob[x]=1
                                        no_of_steps=no_of_steps+1
                                
                            print("*************")
                            print("y",y,p.getBaseVelocity(listbox[y][0]))
                            
                        else:
                            if(boolrob[y]!=1 and no_of_nodes[y]>=0 and steps_list[y]>current_step):
                                theta= p.getEulerFromQuaternion(p.getBasePositionAndOrientation(listbox[y][0])[1])[2]
                                p.resetBaseVelocity(listbox[y][0],[next_node_list[y].vel[0]*math.cos(theta),next_node_list[y].vel[0]*math.sin(theta), 0.0],[0.0,0.0,next_node_list[y].vel[1]])
                                
                if(steps_list[x]<=current_step):#if the edge of the current robot is traversed 
                    p.resetBaseVelocity(listbox[x][0],[0.0,0.0, 0.0],[0.0,0.0,0.0])
                
            p.stepSimulation()
            time.sleep(1./240.)
            current_step=current_step+1
        print(p.getBasePositionAndOrientation(listbox[0][0]))
        
        for k in range(len(current_node_list)):
            current_node_list[k]=next_node_list[k]
        next_node_list=[]
        print("i is ",i)
    p.getBasePositionAndOrientation(listbox[1][0])

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf",[0,0,0],p.getQuaternionFromEuler([0,0,0]),useFixedBase=True)
#Initializing start positions
startPos1 = [0,0,0]
startPos2=[2,0,0]
startPos3=[1,1,0]
#Initializing goal positions
goal1=[2,0,0]
goal2=[0,0,0]
goal3=[1,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
#Initializing robots
box1Id = p.loadURDF("robot.urdf",startPos1, startOrientation,useFixedBase=True,flags=p.URDF_USE_INERTIA_FROM_FILE|p.URDF_USE_MATERIAL_COLORS_FROM_MTL,physicsClientId=0)
box2Id = p.loadURDF("robot.urdf",startPos2, startOrientation,useFixedBase=True,flags=p.URDF_USE_INERTIA_FROM_FILE|p.URDF_USE_MATERIAL_COLORS_FROM_MTL,physicsClientId=0)
box3Id = p.loadURDF("robot.urdf",startPos3, startOrientation,useFixedBase=True,flags=p.URDF_USE_INERTIA_FROM_FILE|p.URDF_USE_MATERIAL_COLORS_FROM_MTL,physicsClientId=0)
listbox=[[box1Id,goal1,startPos1],[box2Id,goal2,startPos2],[box3Id,goal3,startPos3]]#stores list of robots

goal_reached = False

#Obstacle 
obstacleSize = [0.25, 0.25, 0.25]  # Width, Length, Height
obstaclePosition = [3,1,0]  # X, Y, Z position relative to the origin
obstacleOrientation = [0, 0, 0, 1]  # Orientation in Euler angles

obstacleId = p.createCollisionShape(p.GEOM_BOX, halfExtents=obstacleSize)
obstacleVisualShape = p.createVisualShape(p.GEOM_BOX, halfExtents=obstacleSize, rgbaColor=[1, 0, 0, 1])  # Red color
obstacle = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=obstacleId, baseVisualShapeIndex=obstacleVisualShape, basePosition=obstaclePosition, baseOrientation=obstacleOrientation)

#List for storing obstacles
obstacle_list=[obstacle]

#list for storing paths
pathlist=[]

#State exploration for each robot
for box in listbox:
    actual_goal = box[1]
    time.sleep(5)
    goal_reached=False
    x = node(pos = box[2],orn = list(p.getQuaternionFromEuler([0,0,0])),vel = [0,0], time = 0)
    G.add_node(x)
    while goal_reached is not True:#goal_n>(0,0,0)):
        chance = random.uniform(0,100)
        if(0<=int(chance)<90):
            pos_x = random.uniform(0,2)
            pos_y = random.uniform(0,2)
        else:
            pos_x = box[1][0]
            pos_y = box[1][1]

        goal_coordinate = [pos_x,pos_y,0]
        text=p.addUserDebugText("X", goal_coordinate, [1, 0, 0], textSize=1)

        print(goal_coordinate)
        print(goal_reached)
        minimum_state = x
        #Find closest state from the states in the graph to the random goal coordinate 
        for state in G:
            if((euclidean_distance(state.pos,goal_coordinate)) < (euclidean_distance(minimum_state.pos,goal_coordinate))):
                minimum_state = state
        print("*******")
        print(minimum_state.pos,minimum_state.orn)

        p.resetBasePositionAndOrientation(box[0],minimum_state.pos,minimum_state.orn) #moves the robot to the closest state from G 
        print("current pos and orientation after resetting:",p.getBasePositionAndOrientation(box[0]))
        currentPos=minimum_state.pos
        currentOrn=minimum_state.orn
        print(currentPos,currentOrn)
        goal_position_local = transform_and_rotate(minimum_state.pos, minimum_state.orn, goal_coordinate)#stores transformed and rotated goal poistion


        best_edge=find_distance_between_edge_goal(goal_position_local)
        print(best_edge,goal_reached)
        pos=best_edge.pos
        orn=best_edge.orn

        theta = p.getEulerFromQuaternion(currentOrn)[2]


        print(pos,orn,list(p.getEulerFromQuaternion(orn)))
        #simulate the edge
        p.resetBaseVelocity(box[0],[best_edge.vel[0]*math.cos(theta),best_edge.vel[0]*math.sin(theta), 0.0], [0.0,0.0,best_edge.vel[1]])
        print("applied velocities are:",best_edge.vel[0],best_edge.vel[1])
        
        
        flag=1#Used to check if there was a collision in the path of the current edge 
        edgenum=0#stores index of edge from list of edges sorted based on their distance from the goal 
        while(True):
            print("-------")
            print(flag)
            if(flag==0):
                break
            else:#if flag==1 and collision was detected it selects the next best edge  
                flag=0
                best_edge=bundle[edgenum]
                p.resetBaseVelocity(box[0],[best_edge.vel[0]*math.cos(theta),best_edge.vel[0]*math.sin(theta), 0.0], [0.0,0.0,best_edge.vel[1]])
                print("applied velocities are:",best_edge.vel[0],best_edge.vel[1])
            rand_time = best_edge.time
            print("randomly chosen time",rand_time)
            no_of_steps=rand_time * 240
            print("no of steps", int(no_of_steps))
            pathmark=[]#used to trace the path with a marker 
            current_step = 0
            while(current_step!=int(no_of_steps)):
                p.stepSimulation()
                #time.sleep(1./10000.)
                currPos,currOrn=p.getBasePositionAndOrientation(box[0])
                pathmark.append((currPos, currOrn))
                for i in obstacle_list:
                    closest_points = p.getClosestPoints(bodyA=box[0], bodyB=i,distance = 0.1, physicsClientId=physicsClient)
                    if(len(closest_points)!=0):#Collision in the edge
                        edgenum=edgenum+1
                        flag=1
                        p.resetBasePositionAndOrientation(box[0],minimum_state.pos,minimum_state.orn)
                        print("closest pointsss:",closest_points)
                        break
                if flag==1:
                    break
                current_step=current_step+1      
        curPos, curOrn = p.getBasePositionAndOrientation(box[0])#retrieves final position after propogating to edge

        currentPos = list(curPos)
        currentOrn = list(curOrn)
        
        p.removeUserDebugItem(text)
        #add node to the graph
        x = node(pos = currentPos , orn = currentOrn, vel = best_edge.vel, time = best_edge.time)
        G.add_node(x)
        G.add_edge(minimum_state,x)

        #trace the path
        print(euclidean_distance(currentPos,actual_goal))
        if(euclidean_distance(currentPos,actual_goal)<0.5):
            goal_reached = True
        print('*')
        print(pos,orn,currentPos,currentOrn,goal_position_local)
        print("****** ",goal_reached)
    

    for n in G:
        print(n.pos)
    print(n)
    path = []
    path.append(n)
    print(path)
    #starts from goal
    while(n.pos!=box[2]):
        incoming_edges_to_C = G.in_edges(n)#returns incoming edges
        for edge in incoming_edges_to_C:
            print(edge)
            print(edge[0].pos)
            n = edge[0]
            path.append(n)
    print(path)
    for e in path:
        print(e.pos,e.time)
    pathlist.append(path)
    G.clear()
    path=[]
    p.resetBaseVelocity(box[0],[0.0,0.0, 0.0], [0.0,0.0,0.0])

execute_path()
    
p.disconnect()
