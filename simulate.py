import pybullet as p
import math
from datetime import datetime
import time
import pybullet_data
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
def cosd(x):
   return math.cos(x*math.pi/180.0)
def sind(x):
   return math.sin(x*math.pi/180.0)
def acosd(x):
   return math.acos(x)*180.0/math.pi
def asind(x):
   return math.asin(x)*180.0/math.pi
def atan2d(y,x):
   return math.atan2(y,x)*180.0/math.pi


p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
initOrn = p.getQuaternionFromEuler([0, 0, math.pi/3])
robot = p.loadURDF("/Users/adamhung/buttbot/robot.urdf", [0,0,0.1], initOrn,useFixedBase = 0)
numJoints = p.getNumJoints(robot)
#print(numJoints)
for i in range(numJoints):
   print(p.getJointInfo(robot,i))
for i in range(numJoints):
  p.resetJointState(robot, i, 0)
  
L0 = 126
L1 = 136
L2 = 253.6
d = 97
Ln = math.sqrt(L0**2 + d**2)
psi_dif = atan2d(d,L0)

def getPhi():
   mag = acosd(cosd(pitch)*cosd(roll)/(sind(pitch)**2+(cosd(pitch)**2)*(sind(roll)**2)+(cosd(pitch)**2)*(cosd(roll)**2))**(1/2))
   return mag

def IK(x,y):
  phi_mag = getPhi()
  if roll>0:
    phi = 90-phi_mag
  else:
    phi = 90+phi_mag
  psi = phi - psi_dif
  x1 = x - Ln*cosd(psi)
  y1 = y - Ln*sind(psi)
  q2 = acosd((x1**2 + y1**2 - L1**2 - L2**2)/(2*L1*L2))
  q1 = atan2d(y1,x1) + atan2d((L2*sind(q2)),(L1+L2*cosd(q2)))

  #print(' q1: ' + str(q1) + ' q2: ' + str(q2)+ ' x1: ' + str(x1) + ' y1: ' + str(y1)  + ' roll: ' + str(roll))
  q1_passive = 20*math.pi/180
  q2_passive = 80*math.pi/180
  poses = [0.0,0,0,0,(q1*math.pi/180.0),(q2*math.pi/180.0),q1_passive,q2_passive,q1_passive,q2_passive]
  #poses = [0.0,0,0,0,(q1*math.pi/180.0),(q2*math.pi/180.0),(q1*math.pi/180.0),(q2*math.pi/180.0),(q1*math.pi/180.0),(q2*math.pi/180.0)]
  return poses

c = 0
y_ground = -100
cur_x = 200
cur_y = y_ground
step_size = 0.2

p.changeDynamics(robot,-1,lateralFriction = 0.0, mass= 20.0 ,rollingFriction = 0, spinningFriction = 0)
p.changeDynamics(robot,5,lateralFriction = 10000.0, mass= 10.0)
p.changeDynamics(robot,7,lateralFriction = 0.0, mass= 10.0)
p.changeDynamics(robot,9,lateralFriction = 0.0, mass= 10.0)

p.changeDynamics(robot,0,lateralFriction = 0.0, rollingFriction = 0.0, spinningFriction = 0.0)
p.changeDynamics(robot,1,lateralFriction = 0.0, rollingFriction = 0.0, spinningFriction = 0.0)
p.changeDynamics(robot,2,lateralFriction = 0.0, rollingFriction = 0.0, spinningFriction = 0.0)
p.changeDynamics(robot,3,lateralFriction = 0.0, rollingFriction = 0.0, spinningFriction = 0.0)

while 1:
   p.stepSimulation()
   baseOrn = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(robot)[1])
   roll = baseOrn[0] * 180 / math.pi
   #roll = -25
   pitch = 0 #pitch = baseOrn[1] * 180 / math.pi
   print(roll)
   match c:
      case 0:
         cur_x = 200
         cur_y = y_ground
         poses = IK(cur_x, cur_y)
         c = 1
      case 1:
         cur_y = cur_y + step_size
         poses = IK(cur_x, cur_y)
         if cur_y >= y_ground + 20:
            c = 2
      case 2:
         cur_x = cur_x - step_size
         poses = IK(cur_x, cur_y)
         if cur_x <= 100:
            c = 3
      case 3:
         cur_y = cur_y - step_size
         poses = IK(cur_x, cur_y)
         if cur_y <= y_ground:
            c = 4
      case 4:
         cur_x = cur_x + step_size
         poses = IK(cur_x, cur_y)
         if cur_x >= 200:
            c = 1

   for i in range(numJoints):
      if i > 3:
         p.setJointMotorControl2(robot, i, p.POSITION_CONTROL,targetPosition=poses[i])

p.disconnect()


