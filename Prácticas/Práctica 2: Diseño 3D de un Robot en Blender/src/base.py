# Imports from libraries
import pybullet as p
import time
import pybullet_data

# We connect engine with GUI
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# We set gravity (X,Y,Z)
p.setGravity(0,0, -9.8)

# We load a model (plane)
planeId = p.loadURDF("plane.urdf")

# We load new objects, with a position (x,y,z)
# and an orientation given in quaternion (C,X,Y,Z)
startOrientation = p.getQuaternionFromEuler([0, 0, 3.14])
robotId = p.loadURDF("robot_model/urdf/robot_model.urdf",[0, 0, 1.2], startOrientation)
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
cubeId = p.loadURDF("robot_model/cube.urdf",[0, 4, 0], startOrientation)

# Constants
gripper_left_joint = 5
gripper_right_joint = 6
arm_gripper_joint = 4

p.changeDynamics(robotId, gripper_left_joint, jointLowerLimit=0, jointUpperLimit=0.25)
p.changeDynamics(robotId, gripper_right_joint, jointLowerLimit=0, jointUpperLimit=0.25)
p.changeDynamics(robotId, arm_gripper_joint, jointLowerLimit=-1.25, jointUpperLimit=1)

while (1):
    p.stepSimulation()
    time.sleep(1./240.)
