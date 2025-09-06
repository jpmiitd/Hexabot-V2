import pybullet as p
import time
import pybullet_data
import math
physicsClient = p.connect(p.GUI)# p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("urdf/hexabot_v2.urdf",cubeStartPos, cubeStartOrientation)

# for i in range (10000):
#    p.stepSimulation()
#    time.sleep(1./240.)

t0 = time.time()

# Define the oscillation parameters
amplitude_deg = 30
amplitude_rad = math.radians(amplitude_deg)
frequency = 1.0  # Adjust this value to change the speed of oscillation

for i in range(10000):
    p.stepSimulation()

    # Get the current elapsed time
    t = time.time() - t0

    # Calculate the target position for all 24 joints
    for joint_index in range(p.getNumJoints(boxId)):
        # Calculate the oscillating target position using a sine function
        # The 'math.sin' function takes radians as input
        target_pos = amplitude_rad * math.sin(2 * math.pi * frequency * t)

        # Use position control to set the joint's target position
        p.setJointMotorControl2(
            bodyUniqueId=boxId,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_pos,
            force=500  # You might need to adjust this value
        )
    
    time.sleep(1./240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn, flush=True)
print("kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk", flush=True)
print("Total joint: ",p.getNumJoints(boxId), flush=True)
num_joints = p.getNumJoints(boxId)
for i in range(num_joints):
    info = p.getJointInfo(boxId, i)
    print(f"Joint {i}: Name={info[1].decode('utf-8')}, Type={info[2]}")

# Add a delay to keep the program and console open
print("Pausing for 5 seconds to view output...")
time.sleep(5)  # Pause for 5 seconds

p.disconnect()
