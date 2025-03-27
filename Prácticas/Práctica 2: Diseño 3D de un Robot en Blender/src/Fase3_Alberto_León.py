import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI) # connect motor with gui
p.setRealTimeSimulation(1)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0,0, -9.8)

planeId = p.loadURDF("plane.urdf") #load model

startPos = [0, 0, 1.3]
euler_angles = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)

robotId = p.loadURDF("robot_model/urdf/robot_model.urdf",startPos, startOrientation) 

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

wheels_joints = [8, 10, 12, 14]
gripper_joints = [5, 6]
arm_gripper_joint = [4]

gripper_movement = p.addUserDebugParameter("gripper_movement", 0, 5, 0)
arm_gripper_movement = p.addUserDebugParameter("arm_gripper_movement", -25, 25, 0)

# p.changeDynamics(robotId, gripper_left_joint, jointLowerLimit=0, jointUpperLimit=-0.25)
# p.changeDynamics(robotId, gripper_joints[0], jointLowerLimit=0.5, jointUpperLimit=0.75)

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))
    print("Link - %s" % (p.getJointInfo(robotId,j)[12]))

# input("Press enter to start motion")
while (1):

    speed_gripper = p.readUserDebugParameter(gripper_movement)
    speed_arm_gripper = p.readUserDebugParameter(arm_gripper_movement)
    # p.setJointMotorControlArray(robotId, wheels_joints, p.VELOCITY_CONTROL, targetVelocities=[11] * 4)
    
    p.setJointMotorControlArray(robotId,
                              gripper_joints,
                              p.POSITION_CONTROL,
                              targetVelocities=[speed_gripper, speed_gripper])

    p.setJointMotorControlArray(robotId,
                              arm_gripper_joint,
                              p.POSITION_CONTROL,
                              targetVelocities= [speed_arm_gripper])
    
    p.stepSimulation()
    time.sleep(0.01)
