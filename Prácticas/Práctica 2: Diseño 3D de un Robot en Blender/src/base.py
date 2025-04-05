import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0, -9.8)

planeId = p.loadURDF("plane.urdf")

startOrientation = p.getQuaternionFromEuler([0, 0, 3.14])
robotId = p.loadURDF("robot_model/urdf/robot_model.urdf",[0, 0, 1.2], startOrientation)

startOrientation = p.getQuaternionFromEuler([0, 0, 0])
cubeId = p.loadURDF("robot_model/cube.urdf",[0, 4, 0], startOrientation)

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

wheels_joints = [8, 10, 12, 14]
gripper_left_joint = 5
gripper_right_joint = 6
arm_gripper_joint = 4

p.changeDynamics(robotId, gripper_left_joint, jointLowerLimit=0, jointUpperLimit=0.25)
p.changeDynamics(robotId, gripper_right_joint, jointLowerLimit=0, jointUpperLimit=0.25)
p.changeDynamics(robotId, arm_gripper_joint, jointLowerLimit=-1.25, jointUpperLimit=1)

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

while (1):

    p.stepSimulation()
    time.sleep(0.01)
