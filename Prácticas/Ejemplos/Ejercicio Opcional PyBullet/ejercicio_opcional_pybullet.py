import pybullet as p
import pybullet_data
import time

physicsClient = p.connect (p.GUI)
p.setAdditionalSearchPath (pybullet_data.getDataPath())
p.setGravity (0, 0, -9.81)

planeId = p.loadURDF ("plane.urdf")

euler_angles = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0, 0, 1]

robotId = p.loadURDF ("urdf/ejercicio_opcional_pybullet.urdf", startPosition, startOrientation)

frictionIdBase = p.addUserDebugParameter("BASE_jointFriction", 0, 100, 10)
torqueIdBase = p.addUserDebugParameter("BASE_joint torque", -20, 20, -9)
frictionId = p.addUserDebugParameter("ARM_jointFriction", 0, 100, 10)
torqueId = p.addUserDebugParameter("ARM_joint torque", -20, 20, -9)

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

for i in range (10000):

    frictionForceBase = p.readUserDebugParameter(frictionIdBase)
    jointTorqueBase = p.readUserDebugParameter(torqueIdBase)
    frictionForce = p.readUserDebugParameter(frictionId)
    jointTorque = p.readUserDebugParameter(torqueId)

    #set the joint friction
    p.setJointMotorControl2(robotId, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForceBase)
    #apply a joint torque
    p.setJointMotorControl2(robotId, 0, p.TORQUE_CONTROL, force=jointTorqueBase)
    #set the joint friction
    p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)
    #apply a joint torque
    p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=jointTorque)
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()