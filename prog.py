import pybullet as p
import time
import pybullet_data
import random
import numpy as np

def randomize_wheel_velocities():
    linear_velocity = random.uniform(-0.22,0.22) 
    angular_velocity_z = random.uniform(-1.0,1.0)  
    return linear_velocity,angular_velocity_z
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)
planeId = p.loadURDF("plane.urdf",[0,0,0],p.getQuaternionFromEuler([0,0,0]),useFixedBase=True)

startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("robot.urdf",startPos, startOrientation,useFixedBase=True,flags=p.URDF_USE_INERTIA_FROM_FILE|p.URDF_USE_MATERIAL_COLORS_FROM_MTL,physicsClientId=0)
# Initialize an empty list to store the path
path = []

velocities = []  # Stores tuples of (left_velocity, right_velocity) for each iteration
edge=[]#stores each edge info
bundle=[]#stores bundle of edges
for i in range (10):
    edge=[]
    velocities=[]
    linear_velocity,angular_velocity_z = randomize_wheel_velocities()
    print("this is the lv and av respectively",linear_velocity,angular_velocity_z)
    velocities.append((linear_velocity, angular_velocity_z))  # Store current velocities
    edge.extend(velocities)#
    p.resetBaseVelocity(boxId, [linear_velocity,0.0, 0.0], [0.0,0.0,angular_velocity_z])
    rand_time = random.uniform(1, 2)
    print("randomly chosen time",rand_time)
    edge.append(rand_time)#
    no_of_steps=rand_time * 240
    print("no of steps", int(no_of_steps))
    current_step = 0
    while(current_step!=int(no_of_steps)):
        p.stepSimulation()
        
        currPos,currOrn=p.getBasePositionAndOrientation(boxId)
        path.append((currPos, currOrn))
        current_step=current_step+1
    print("Edge ",i)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    print("Position:",cubePos,"Orientation:",cubeOrn)
    edge.append(cubePos)
    edge.append(cubeOrn)
    bundle.append(edge)
    
    for i in range(len(path)-1):
        pos1, orn1 = path[i]
        pos2, orn2 = path[i+1]
        p.addUserDebugLine(pos1, pos2, [1, 0, 0], 2, 0)
    path=[]
    position=[1,0,0]
    positiono=[0,0,0]
    orientation=p.getQuaternionFromEuler([0,0,0])
    p.resetBasePositionAndOrientation(boxId,positiono,orientation)
   

print("Velocities:", velocities)
print("Bundle",bundle)
bundle_arr=np.array(bundle,dtype=object)
np.save("bundle.npy",bundle_arr)

p.disconnect()