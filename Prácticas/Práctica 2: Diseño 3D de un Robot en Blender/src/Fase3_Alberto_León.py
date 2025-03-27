import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0, -9.8)

planeId = p.loadURDF("plane.urdf")

startPos = [0, 0, 1.2]
startOrientation = p.getQuaternionFromEuler([0, 0, 3.14])
robotId = p.loadURDF("robot_model/urdf/robot_model.urdf",startPos, startOrientation) 

startPos = [0, 4, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
cubeId = p.loadURDF("robot_model/cube.urdf",startPos, startOrientation)

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

"""
0 - arm_base_link_joint
1 - arm_column_link_joint
2 - arm_1_link_joint
3 - arm_2_link_joint
4 - arm_gripper_link_joint
5 - gripper_left_link_joint
6 - gripper_right_link_joint
7 - subjection_down_left_wheel_link_joint
8 - wheel_down_left_link_joint
9 - subjection_down_right_wheel_link_joint
10 - wheel_down_right_link_joint
11 - subjection_up_left_wheel_link_joint
12 - wheel_up_left_link_joint
13 - subjection_up_right_wheel_link_joint
14 - wheel_up_right_link_joint
"""

wheels_joints = [8, 10, 12, 14]
gripper_left_joint = 5
gripper_right_joint = 6
arm_gripper_joint = 4

"""
gripper_left_movement = p.addUserDebugParameter("gripper_left_movement", 0, 5, 0)
gripper_right_movement = p.addUserDebugParameter("gripper_right_movement", 0, 5, 0)
arm_gripper_movement = p.addUserDebugParameter("arm_gripper_movement", -30, 25, 0)
"""

p.changeDynamics(robotId, gripper_left_joint, jointLowerLimit=0, jointUpperLimit=0.25)
p.changeDynamics(robotId, gripper_right_joint, jointLowerLimit=0, jointUpperLimit=0.25)
p.changeDynamics(robotId, arm_gripper_joint, jointLowerLimit=-1.25, jointUpperLimit=1)

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

# input("Press enter to start motion")
while (1):

    """
    speed_left_gripper = p.readUserDebugParameter(gripper_left_movement)
    speed_right_gripper = p.readUserDebugParameter(gripper_right_movement)
    speed_arm_gripper = p.readUserDebugParameter(arm_gripper_movement)
    """
    
    # p.setJointMotorControlArray(robotId, wheels_joints, p.VELOCITY_CONTROL, targetVelocities=[11] * 4)
    
    """
    p.setJointMotorControlArray(robotId,
                                gripper_left_joint,
                                p.POSITION_CONTROL,
                                targetVelocities=[speed_left_gripper])
                                
    p.setJointMotorControlArray(robotId,
                                gripper_right_joint,
                                p.POSITION_CONTROL,
                                targetVelocities=[speed_right_gripper])

    p.setJointMotorControlArray(robotId,
                                arm_gripper_joint,
                                p.POSITION_CONTROL,
                                targetVelocities= [speed_arm_gripper])
    """
    
    p.stepSimulation()
    time.sleep(0.01)
