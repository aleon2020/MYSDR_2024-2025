# Imports from libraries
import pybullet as p
import pybullet_data
import time

# We connect engine with GUI
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# We set gravity (X,Y,Z)
p.setGravity(0, 0, -9.8)

# We load a model (plane)
planeID = p.loadURDF("plane.urdf")

euler_angles = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)

# We load new objects, with a position (x,y,z)
# and an orientation given in quaternion (C,X,Y,Z)
huskyId = p.loadURDF("husky/husky.urdf", [0, 0, 0], startOrientation)
rampId = p.loadURDF("urdf/ramp.urdf", [10, 0, 0], startOrientation)
barrierId = p.loadURDF("urdf/barrier.urdf", [17, 1.5, 0], startOrientation)
finishlineId = p.loadURDF("urdf/finish_line.urdf", [20, 0, 0], startOrientation)

joints = [2, 3, 4, 5]

while (1):

    p.stepSimulation()
    time.sleep(1./240.)

    speed = 15
    torque = 50

    p.setJointMotorControlArray(huskyId,
                                joints,
                                p.VELOCITY_CONTROL,
                                targetVelocities=[speed, speed, speed, speed],
                                forces=[torque, torque, torque, torque])
    
    p.resetDebugVisualizerCamera(cameraDistance=5, 
                                 cameraYaw=p.getDebugVisualizerCamera()[8], 
                                 cameraPitch=p.getDebugVisualizerCamera()[9], 
                                 cameraTargetPosition=p.getBasePositionAndOrientation(huskyId)[0])

    if p.getBasePositionAndOrientation(huskyId)[0][0] >= 20:
        break

p.disconnect()