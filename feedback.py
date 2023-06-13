
import numpy as np
from pybullet_controller import RobotController
import pybullet as p
import matplotlib.pyplot as plt


robot = RobotController(robot_type='ur5')
robot.createWorld(GUI=True)

q1 = np.array(robot.solveInversePositionKinematics([0.3,0.3,2,-1.57,0,0]))
q2 = np.array(robot.solveInversePositionKinematics([0.4,0.4,1.883,-1.57,0,0]))
q3 = np.array(robot.solveInversePositionKinematics([0.5,0.4,1.883,-1.57,0,0]))
q4 = np.array(robot.solveInversePositionKinematics([0.4,0.5,1.883,-1.57,0,0]))
#Putting Kp = 900 , Kd =30 

robot.setJointPosition(q1)
robot.feedback(q1,q2)
robot.feedback(q2,q3)
robot.feedback(q3,q4)
robot.feedback(q4,q1)

#plt.show()
