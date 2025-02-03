import pybullet as p
import time
import pybullet_data
import numpy as np
import time
from simple_pid import PID
import math


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(1)

planeId = p.loadURDF("plane.urdf")

robotId = p.loadURDF("./urdf/my_arm.urdf", [0,0,1.0])
#robotId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))


for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))


# Solo queremos que la cinemática inversa actue sobre los primeros 3 JOINTS
robotEndEffectorIndex=3


pos_target = [1,1,1]
jointPoses = p.calculateInverseKinematics(robotId, robotEndEffectorIndex, pos_target)
print(jointPoses)
p.addUserDebugText("X", pos_target, [1,0,0], 1)


for i in range(len(jointPoses)):
    p.setJointMotorControl2(bodyIndex=robotId,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i],
                                    targetVelocity=0,
                                    force=500,
                                    positionGain=0.03,
                                    velocityGain=1)
while(1):

    # No es necesario, solo para ver más despacio el movimiento.
    time.sleep(1./20.)
    pass
