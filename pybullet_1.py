import pybullet
import time
import pybullet_data

physicsClient = pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
planeID = pybullet.loadURDF("plane.URDF")
robot = pybullet.loadURDF("C:\\Users\\17864\\source\\kuka_experimental\\kuka_lbr_iiwa_support\\urdf\\lbr_iiwa_14_r820.urdf", [0,0,0], useFixedBase=1)

######################################
#print(pybullet.getNumJoints(robot))

#position, orientation = pybullet.getBasePositionAndOrientation(robot)
#print(position, orientation)

#print(pybullet.getJointInfo(robot, 7))
#####################################

pybullet.setGravity(0, 0, -9.81)
pybullet.setRealTimeSimulation(0)

#####################################
#Inverse Kinematic
orientation = pybullet.getQuaternionFromEuler([3.14, 0, 0])
position = [0.1, 0.1, 0.2]

targetpositionJoints = pybullet.calculateInverseKinematics(robot, 7, position, targetOrientation=orientation)

#####################################
pybullet.setJointMotorControlArray(robot, range(7), pybullet.POSITION_CONTROL, targetPositions=targetpositionJoints)

for i in range(100):
    pybullet.stepSimulation()
    time.sleep(1./10.)