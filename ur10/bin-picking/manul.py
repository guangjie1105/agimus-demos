# Copyright 2021 CNRS - Airbus SAS
# Author: Florent Lamiraux
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from cgi import test
from http.client import REQUESTED_RANGE_NOT_SATISFIABLE
from importlib.resources import path
import sys, argparse, numpy as np, time, rospy
from unicodedata import name
from turtle import forward
from math import pi, sqrt
from hpp import Transform
from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import Robot, \
    createContext, newProblem, ProblemSolver, ConstraintGraph, \
    ConstraintGraphFactory, Rule, Constraints, CorbaClient, SecurityMargins
from hpp.gepetto import PathPlayer
from hpp.gepetto.manipulation import ViewerFactory
from tools_hpp import RosInterface, PathGenerator
#from manipulation import Ground, Bin


UseAprilTagPlank = False
useFOV = False

class PartPlaque:
    urdfFilename = "package://agimus_demos/urdf/plaque-tubes-with-table.urdf"
    srdfFilename = "package://agimus_demos/srdf/plaque-tubes.srdf"
    rootJointType = "freeflyer"

class AprilTagPlank:
    urdfFilename = "package://agimus_demos/urdf/april-tag-plank.urdf"
    srdfFilename = "package://agimus_demos/srdf/april-tag-plank.srdf"
    rootJointType = "freeflyer"
class Bin (object):
  rootJointType = 'freeflyer'
  packageName = 'agimus_demos'
  meshPackageName = 'agimus_demos'
  urdfName = 'bin-picking-part'
  urdfSuffix = ""
  srdfSuffix = ""

class Ground:
    urdfFilename = "package://agimus_demos/urdf/ground.urdf"
    srdfFilename = "package://agimus_demos/srdf/ground.srdf"
    rootJointType = "anchor"


class Box:
    urdfFilename = "package://agimus_demos/urdf/box.urdf"
    srdfFilename = "package://agimus_demos/srdf/box.srdf"
    rootJointType = "anchor"

class Lip:
    urdfFilename = "package://agimus_demos/urdf/lip.urdf"
    srdfFilename = "package://agimus_demos/srdf/lip.srdf"
    rootJointType = "freeflyer"

# parse arguments
defaultContext = "corbaserver"
p = argparse.ArgumentParser (description=
                             'Initialize demo of UR10 pointing')
p.add_argument ('--context', type=str, metavar='context',
                default=defaultContext,
                help="identifier of ProblemSolver instance")
args = p.parse_args ()

joint_bounds = {}
def setRobotJointBounds(which):
    for jn, bound in jointBounds[which]:
        robot.setJointBounds(jn, bound)

try:
    import rospy
    Robot.urdfString = rospy.get_param('robot_description')
    print("reading URDF from ROS param")
except:
    print("reading generic URDF")
    from hpp.rostools import process_xacro, retrieve_resource
    Robot.urdfString = process_xacro\
      ("package://agimus_demos/urdf/ur10_bin-picking_sim.urdf.xacro",
       "transmission_hw_interface:=hardware_interface/PositionJointInterface")
Robot.srdfString = ""

loadServerPlugin (args.context, "manipulation-corba.so")
newProblem()
client = CorbaClient(context=args.context)
#client.basic._tools.deleteAllServants()
client.manipulation.problem.selectProblem (args.context)
def wd(o):
    from hpp.corbaserver import wrap_delete
    return wrap_delete(o, client.basic._tools)

robot = Robot("robot", "ur10e", rootJointType="anchor", client=client)
crobot = wd(wd(robot.hppcorba.problem.getProblem()).robot())
ps = ProblemSolver(robot)
p = robot.hppcorba.problem.getProblem()
cdistance = p.getDistance()
croadmap = ps.client.manipulation.problem.createRoadmap(cdistance, crobot)
print("Robot loaded")
robot.opticalFrame = 'camera_color_optical_frame'
ps.loadPlugin("manipulation-spline-gradient-based.so")
ps.addPathOptimizer("EnforceTransitionSemantic")
ps.addPathOptimizer("SimpleTimeParameterization")
# ps.selectConfigurationShooter('Gaussian')
# ps.setParameter('ConfigurationShooter/Gaussian/center', 12*[0.] + [1.])
# ps.setParameter('ConfigurationShooter/Gaussian/standardDeviation', 0.25)
ps.setParameter('SimpleTimeParameterization/order', 2)
ps.setParameter('SimpleTimeParameterization/maxAcceleration', .5)
ps.setParameter('SimpleTimeParameterization/safety', 0.95)

# Add path projector to avoid discontinuities
ps.selectPathProjector ("Progressive", .05)
ps.selectPathValidation("Graph-Progressive", 0.01)
vf = ViewerFactory(ps)
vf.loadEnvironmentModel (Ground, 'ground')
vf.loadEnvironmentModel  (Box, 'box')    #as object because later need to use its link
#vf.loadObjectModel  (Lip, 'lip')



## Shrink joint bounds of UR-10
#
jointBounds = dict()
jointBounds["default"] = [ (jn, robot.getJointBounds(jn)) \
                           if not jn.startswith('ur10/') else
                           (jn, [-pi, pi]) for jn in robot.jointNames]
jointBounds["limited"] = [('ur10e/shoulder_pan_joint', [-pi, pi]),
  ('ur10e/shoulder_lift_joint', [-pi, pi]),
  ('ur10e/elbow_joint', [-3.1, 3.1]),
  ('ur10e/wrist_1_joint', [-3.2, 3.2]),
  ('ur10e/wrist_2_joint', [-3.2, 3.2]),
  ('ur10e/wrist_3_joint', [-3.2, 3.2])]

setRobotJointBounds("limited")
## Remove some collision pairs
#
ur10JointNames = list(filter(lambda j: j.startswith("ur10/"), robot.jointNames))
ur10LinkNames = [ robot.getLinkNames(j) for j in ur10JointNames ]

## Load P72
#[1., 0, 0.8,0,0,-sqrt(2)/2,sqrt(2)/2]
if UseAprilTagPlank:
    Part = AprilTagPlank
else:
    Part = Bin
vf.loadObjectModel (Part, "part")
robot.setJointBounds('part/root_joint', [-6,6, -6 ,6, -0.05, 8])
print("Part loaded")

vf.loadObjectModel (Lip, 'lid')###change the name.
robot.setJointBounds('lid/root_joint', [-6,6, -6 ,6, -0.05, 8])
ps.createLockedJoint('locked-lid','lid/root_joint',[0,0,0.11,0,0,0,1])
print('lid loaded')
robot.client.manipulation.robot.insertRobotSRDFModel\
    ("ur10e", "package://agimus_demos/srdf/ur10_robot.srdf")

partPose = [1.3, 0, 0,0,0,-sqrt(2)/2,sqrt(2)/2]

## Define initial configuration
q0 = robot.getCurrentConfig()
print("q0",q0)
# set the joint match with real robot
q0[:6] = [0, -pi/2, 0.89*pi,-pi/2, -pi, 0.5]
r = robot.rankInConfiguration['part/root_joint']
if UseAprilTagPlank:
    # Initial configuration of AprilTagPlank
    q0[r:r+7] = [1.3, 0, 0, 0, 0, -1, 0]
else:
    q0[r:r+7] = partPose
#q3=q0[::]
#q3[2]=0.3
#ps.setInitialConfig (q0)
#ps.addGoalConfig (q3)
## Home configuration
q_home = [-3.415742983037262e-05, -1.5411089223674317, 2.7125137487994593, -1.5707269471934815, -3.141557280217306, 6.67572021484375e-06, 1.3, 0, 0, 0, 0, -0.7071067811865476, 0.7071067811865476]
## Calibration configuration: the part should be wholly visible
q_calib = [1.5707, -3, 2.5, -2.8, -1.57, 0.0, 1.3, 0.0, 0.0, 0.0, 0.0, -0.7071067811865476, 0.7071067811865476]
## PointCloud position : the part should fit the field of view
q_pointcloud = [1.4802236557006836, -1.7792146009257812, 2.4035003821002405, -0.9398099416545411, 1.5034907341003418, -3.1523403135882773, 1.2804980083956572, 0.11300105405990518, -0.031348192422114174, -0.008769144315009561, 0.004377057629846714, -0.7073469546030107, 0.7067985775935985]
q_pointcloud2 = [1.480271816253662, -1.4792188129820762, 2.403522316609518, -0.9397409719279786, 1.503467082977295, -3.152398173009054, 1.166362251685465, 0.18398354959470994, -0.040275835859414855, -0.011115686791169354, 0.00903223476857969, -0.701584375075136, 0.7124424361958492]
q_pc1 = [1.4802236557006836, -2.7792393169798792, 2.6035669485675257, 0.06014220296826167, 1.5035147666931152, -3.1523261705981653, 1.1430909403610665, 0.22893782415973665, -0.04789935128099243, -0.0033794390562739483, 0.008636413673842097, -0.6985826418521135, 0.7154692755481825]
q_pc2 = [1.480247974395752, -2.7792188129820765, 2.6035461584674278, -0.13980153024707037, 1.5035266876220703, -3.1523383299456995, 1.1138463304333714, 0.23051956151197342, -0.040682700437678854, -0.00938303410217683, 0.013303913345295534, -0.7001874363145009, 0.7137734364544993]



def norm(quaternion):
    return sqrt(sum([e*e for e in quaternion]))

gripper = 'ur10e/gripper'
## Create specific constraint for a given handle
#  Rotation is free along x axis.
#  Note that locked part should be added to loop edge.
def createFreeRxConstraintForHandle(handle):
    name = gripper + ' grasps ' + handle
    handleJoint, jMh = robot.getHandlePositionInJoint(handle)
    gripperJoint, jMg = robot.getGripperPositionInJoint(gripper)
    ps.client.basic.problem.createTransformationConstraint2\
        (name, gripperJoint, handleJoint, jMg, jMh,
         [True, True, True, False, True, True])
    # pregrasp
    shift = 0.13
    M = Transform(jMg)*Transform([shift,0,0,0,0,0,1])
    name = gripper + ' pregrasps ' + handle
    ps.client.basic.problem.createTransformationConstraint2\
        (name, gripperJoint, handleJoint, M.toTuple(), jMh,
         [True, True, True, False, True, True])

## Build constraint graph
def createConstraintGraph():
    #all_handles = ps.getAvailable('handle')
    #part_handles = list(filter(lambda x: x.startswith("part/"), all_handles))
    #objContactSurfaces =[['part/bottom',]]
    #envSurfaces=['box/box_surface','ground/surface']


    graph = ConstraintGraph(robot, 'graph2')
    #rules = [Rule ([""], [""], True)]
    factory = ConstraintGraphFactory(graph)

    
    
    #Constraint by hand,because the pose of part is offered by vision system so do not need placement constraint
    
    graph.createNode(['intersec'],priority = 2 )
    graph.createNode(['preplace'],priority = 3)
    graph.createNode(['pregrasp'],priority = 2)
    graph.createNode(['free'],priority = 1)
    graph.createNode(['grasps'],priority = 2)
    #graph.createNode(['vertical'],priority = 0)

    graph.createEdge ('free', 'pregrasp', 'approach-part', 1, 'free')
    graph.createEdge ('pregrasp', 'free', 'move-gripper-away', 1, 'free')
    graph.createEdge ('pregrasp', 'intersec', 'grasp-part', 1, 'free')
    graph.createEdge ('intersec', 'pregrasp', 'move-gripper-up', 1, 'free')
    graph.createEdge ('intersec', 'preplace', 'take-part-up', 1, 'grasps')
    graph.createEdge ('preplace', 'intersec', 'put-part-down', 1, 'grasps')
    graph.createEdge ('preplace', 'grasps', 'take-part-away', 1, 'grasps')
    graph.createEdge ('grasps', 'preplace', 'approach-box', 1, 'grasps')
    graph.createEdge ('free', 'free', 'transit', 1, 'free')
    graph.createEdge ('grasps', 'grasps', 'transfer', 1, 'grasps')
    ps.createTransformationConstraint ('grasp', 'ur10e/gripper', 'part/handle_link',
                                   [0.02,0,0.0,0, 0, 0, 1],
                                   [True, True, True, True,True,True])
    ps.createTransformationConstraint ('part-above-box','part/base_link','',
                                   [0,0,-0.35,0, 0, 0, 1],
                                   [False, False, True, False, False,False])
    ps.createTransformationConstraint ('placement/complement', '','part/base_link',
                                   [0,0,0,0, 0, 0, 1],
                                   [True, True,True, True,True, True,])
    ps.setConstantRightHandSide('placement/complement', False)
    ps.createTransformationConstraint ('movement-vertical', 'part/base_link','',
                                   [0,0,0.095,0, 0, 0, 1],
                                   [True, True, False, True,True,True,])
    ps.setConstantRightHandSide ('movement-vertical', False) 
    ps.createTransformationConstraint ('gripper-above-part', 'ur10e/gripper', 'part/handle_link',
                                   [0.18,0,0,0,0,0,1], [True,True,True,True,True,True])    ##x y z w
    #ps.createTransformationConstraint ('vertical','part/base_link','',
     #                              [0,0,-0.35,0, 0, 0, 1],
      #                             [False, False, False, True, True, False])
    ps.createTransformationConstraint ('gripper-vertical', 'ur10e/gripper', 'part/handle_link',
                                   [0.2,0,0,0,0,0,1], [False, True,True, False, True, True,])
    #ps.setConstantRightHandSide('gripper-vertical', False)

    #Node constraints
    #graph.addConstraints(node='vertical',
                       # constraints = Constraints(numConstraints=['vertical'])) 
    graph.addConstraints(graph = True,
                        constraints = Constraints(numConstraints=['locked-lid']))
    graph.addConstraints(node='preplace',
                        constraints = Constraints(numConstraints=['part-above-box','grasp']))
   
    graph.addConstraints (node='intersec',
                      constraints = Constraints (numConstraints = ['grasp']))
    graph.addConstraints (node='pregrasp',
                      constraints = Constraints (numConstraints = ['gripper-above-part']))
    graph.addConstraints (node='grasps',
                      constraints = Constraints (numConstraints = ['grasp']))
    #graph.addConstraints (node='free',
     #                 constraints = Constraints (numConstraints = ['vertical']))
    #make part extrem vertical fix bug of 'take-part-up',because node 'part-above-box'ask for vertical,make it reachable

    #Edge constraints
    graph.addConstraints (edge='transit', constraints = \
                      Constraints (numConstraints = ['placement/complement']))
    graph.addConstraints (edge='approach-part', constraints = \
                      Constraints (numConstraints = ['placement/complement']))  ##dont want the pos of ball change  
    graph.addConstraints (edge='move-gripper-away', constraints = \
                      Constraints (numConstraints = ['placement/complement']))
    graph.addConstraints (edge='grasp-part', constraints = \
 	                     Constraints (numConstraints = ['placement/complement','gripper-vertical']))
    graph.addConstraints (edge='move-gripper-up', constraints = \
 	                     Constraints (numConstraints = ['placement/complement','gripper-vertical']))
    graph.addConstraints (edge='take-part-up', constraints = \
 	                     Constraints (numConstraints = ['movement-vertical']))
    graph.addConstraints (edge='put-part-down', constraints = \
 	                     Constraints (numConstraints = ['movement-vertical']))
   #############

    n = norm([-0.576, -0.002, 0.025, 0.817])
    
    sm = SecurityMargins(ps, factory, ["ur10e", "part"])
    sm.setSecurityMarginBetween("ur10e", "part", 0.015)
    sm.setSecurityMarginBetween("ur10e", "ur10e", 0)
    sm.setSecurityMarginBetween("ur10e", "lip", 0.05)
    sm.defaultMargin = 0.01
    sm.apply() # try
    ##deactive collision between "lip" and"wrist_3_joint"
    cproblem = wd(ps.client.basic.problem.getProblem())
    cgraph = wd(cproblem.getConstraintGraph())
    all_name = [key for key in graph.edges]
    edge_active =  ['transfer','approach-box','put-part-down','move-gripper-up','move-gripper-away']
    name =  [i for i in all_name if i not in edge_active]
    print(name)
    ## deactive lid collision test through the edges pick up part 
    for i in name:
        cedge = wd(cgraph.get(graph.edges[i]))
        cedge.setSecurityMarginForPair(robot.jointNames.index('lid/root_joint')+1, \
        robot.jointNames.index('ur10e/wrist_3_joint')+1 ,float('-inf')) 
        print(i)
    graph.initialize()  # after set Security margins
    # Set weights of levelset edges to 0
    for e in graph.edges.keys():
        if e[-3:] == "_ls" and graph.getWeight(e) != -1:
            graph.setWeight(e, 0)
    return graph


graph = createConstraintGraph()

try:
    v = vf.createViewer()
    #v(q0)
    pp = PathPlayer(v)
except:
    print("Did you launch the GUI?")

ri = None
ri = RosInterface(robot)
q_init = ri.getCurrentConfig(q0)
print("q_int",q_init)
# q_init = q0 #robot.getCurrentConfig()
pg = PathGenerator(ps, graph, ri, v, q_init)
#pg.inStatePlanner.setEdge('Loop | f')
pg.testGraph()
NB_holes = 5 * 7
NB_holes_total = 44
hidden_holes = [2,10,11,12,14,16,24,33]  #remove 0
holes_to_do = [i for i in range(NB_holes) if i not in hidden_holes]
pg.setIsClogged(None)
ps.setTimeOutPathPlanning(10)
pg.setConfig("home", q_home)
pg.setConfig("calib", q_calib)
pg.setConfig("pointcloud", q_pointcloud)
pg.setConfig("pointcloud2", q_pointcloud2)
pg.setConfig("pointcloud_bas", q_pc1)
pg.setConfig("pointcloud_haut", q_pc2)
ps.resetGoalConfigs
if useFOV:
    def configHPPtoFOV(q):
        return q[:6] + q[-7:]

    from ur10_fov import RobotFOV, RobotFOVGuiCallback, Feature, Features
    ur10_fov = RobotFOV(urdfString = Robot.urdfString,
                        fov = np.radians((69.4, 52)),
                        geoms = [],
                        optical_frame = "camera_color_optical_frame",
                        group_camera_link = "robot/ur10e/ref_camera_link",
                        camera_link = "ref_camera_link",
                        modelConfig = configHPPtoFOV)
    robot.setCurrentConfig(q_init)
    # Add Plaque model in the field of view object
    # to test if the plaque itself obstructs the view to the features
    oMh, oMd = robot.hppcorba.robot.getJointsPosition(q_init, ["universe", "part/base_link"])
    fMm = (Transform(oMh).inverse() * Transform(oMd)).toTuple()
    ur10_fov.appendUrdfModel(PartPlaque.urdfFilename, "universe",
        fMm, prefix="part/")
    feature_list = []
    for i in range(1, NB_holes_total+1):
        feature_list.append( Feature('part/hole_' + str(i).zfill(2) + '_link', 0.003) )
    featuress = [Features(feature_list, 2, 0.005, 0)]
    ur10_fov_gui = RobotFOVGuiCallback(robot, ur10_fov, featuress, modelConfig = configHPPtoFOV)
    # Display Robot Field of view.
    #vf.guiRequest.append( (ur10_fov.loadInGui, {'self':None}))
    # Display visibility cones.
    vf.addCallback(ur10_fov_gui)
    isClogged = lambda x : ur10_fov.clogged(x, robot, featuress)
    pg.setIsClogged(isClogged)
    visibleFeatures = lambda x : ur10_fov.visible(x, robot, featuress)
### DEMO

def getDoableHoles():
    doableHoles = []
    non_doableHoles = []
    pg.testGraph()
    for i in range(NB_holes_total):
        if pg.isHoleDoable(i):
            doableHoles.append(i)
        else:
            non_doableHoles.append(i)
    print("Doable holes : ", doableHoles)
    print("Non doable holes : ", non_doableHoles)
    return doableHoles, non_doableHoles

def doDemo():
    NB_holes_to_do = 7
    demo_holes = range(NB_holes_to_do)
    pids, qend = pg.planDeburringPaths(demo_holes)

holist = [7,8,9,42,43,13]
#v(q_init)

""" res2 ,res4 = False,False
while not (res2 and res4):
    q = robot.shootRandomConfig ()
    res1,q1,err = graph.applyNodeConstraints ('free', q)
    q = robot.shootRandomConfig ()
    res3,q2,err = graph.applyNodeConstraints ('free', q)
    if not res1 and res3:
        continue
    res2, msg = pg.robot.isConfigValid(q1)
    res4, msg = pg.robot.isConfigValid(q2)
ps.setInitialConfig (q1)
ps.addGoalConfig (q2)
v(q1) """




""" forward_tool = [[],[]]
for i in range(10000):
    res1,res = False,False
    while not (res and q6[8]<0.03):
        qrand = robot.shootRandomConfig()
        res1, q6, error = graph.applyNodeConstraints ('intersec', qrand)
        if not res1:
            continue
        res = crobot.setCurrentConfiguration(q6)
    forward_tool[0].append(q6[6])
    forward_tool[1].append(q6[7])
x_max = max(forward_tool[0])
x_min = min(forward_tool[0])
y_max = max(forward_tool[1])
y_min = min(forward_tool[1])   ##attation the area is rectangle so is not very percise """



####"#### 
### This part is to read and transform the data from vision system

import transforms3d
import pandas as pd
data = pd.read_csv('E0_0d.txt',header=None)
colo = data.shape[1]                            
row = data.shape[0]
empty = pd.DataFrame([0]*141).T
data = data.append(empty,ignore_index=True)   #remove empty '0' index
for i in range(data.shape[1]):
    mean  = data[i].mean()
    data.iloc[row,i] = mean

#take average from dataframe add to the last row

list = data.iloc[50].values.tolist()    # read as list

for i in range(len(list)):
    if list[i] == 0:
        del list[i:len(list)]      #remove rest value when meet 0
        break           #avoid index out of range

coor_part = [list[i:i+6] for i in range(0,len(list),6)]    #split each 6 ele

## convert from euler to quaternion
for i in range (len(coor_part)):
    rx = coor_part[i][3]/180*np.pi
    ry = coor_part[i][4]/180*np.pi
    rz = coor_part[i][5]/180*np.pi
    quater =  transforms3d.euler.euler2quat(rx,ry ,rz ,axes='sxyz')
    coor_part[i][-3:] = quater # w x y z


#Transform a given input pose of rigid body from one fixed frame to another
import rospy
from geometry_msgs.msg import Pose

import tf2_ros
import tf2_geometry_msgs  #    poseStamped  >> pose --header


def transform_pose(input_pose, from_frame, to_frame):      # w x y z 
    # **Assuming /tf topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame   #Frame this data is associated with
    pose_stamped.header.stamp = rospy.Time(0)    # return the latest available data for a specific transform

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise


# Test Case
part_world = []

for i in range (len(coor_part)):
    my_pose = Pose()
    my_pose.position.x =coor_part[i][0]/1000
    my_pose.position.y = coor_part[i][1]/1000
    my_pose.position.z = coor_part[i][2]/1000
    my_pose.orientation.x = coor_part[i][4]
    my_pose.orientation.y = coor_part[i][5]
    my_pose.orientation.z = coor_part[i][6]
    my_pose.orientation.w = coor_part[i][3]

    transformed_pose = transform_pose(my_pose, "cam_link", "world")
    #HPP--x y z w
    coor = [(transformed_pose.position.x)+0.08,transformed_pose.position.y,(transformed_pose.position.z)+0.01,\
        transformed_pose.orientation.x,transformed_pose.orientation.y,transformed_pose.orientation.z,\
            transformed_pose.orientation.w]
    part_world.append(coor)

go = []
for i in range(len(part_world)):
    neutral_pose = [0, -pi/2, 0.89*pi,-pi/2, -pi, 0.5]
    conf = neutral_pose + part_world[i]+[0,0,0.1,0,0,0,1]   ##arm + part + lid
    go.append(conf)
    v(conf)
    time.sleep(0.5)
#######
## 


##test

""" final = [0, -1.5707963267948966, 2.796017461694916, -1.5707963267948966, -3.141592653589793,\
            0.5, 0.66, 0.7, 3e-06, 0, 0, 0, 1]
res2 = ps.client.manipulation.problem.applyConstraints(graph.nodes['free'],final)
if not res2[0]:
    raise Exception ('Goal configuration could not be projected.')
final = res2[1]  """

#ps.addPathOptimizer("SimpleTimeParameterization")
ps.setTimeOutPathPlanning(30)

#go = [[0, -1.5707963267948966, 2., -1.5707963267948966, 0, 0.5, 1.0432209178713296, 0.20583790467945282, 0.2522109607843137, 0.010857436013281468, -0.007109644686263878, 0.99955548479711, -0.026840302674650103], [0, -1.5707963267948966, 2., -1.5707963267948966, 0, 0.5, 1.0426075142635602, 0.028049295747804535, 0.25237582352941174, 0.017230991582646883, -0.0030211363131261966, 0.999758171303341, -0.013325335892780057], [0, -1.5707963267948966, 2., -1.5707963267948966, 0, 0.5, 1.0397627870322104, -0.1266349005619249, 0.2505982156862745, 0.01367315419487967, -0.01359126426655792, 0.9995505107663977, -0.02295863272568867], [0, -1.5707963267948966, 2., -1.5707963267948966, 0, 0.5, 0.9296967464201046, 0.2067981065192473, 0.25364501960784314, 0.004667391514166578, -0.006208225942286833, 0.9999081748890755, -0.011104736696600913], [0, -1.5707963267948966, 2., -1.5707963267948966, 0, 0.5, 0.9349991154141712, 0.039512694934721385, 0.2544894705882353, 0.007663720375115889, -0.005089128671218016, 0.9995081203537567, -0.02998141935065666], [0, -1.5707963267948966, 2., -1.5707963267948966, 0, 0.5, 0.9278283302527075, -0.1286156436428715, 0.25210409803921563, 0.007780960745240349, -0.014421554984323032, 0.999759562753898, -0.014570246526740528], [0, -1.5707963267948966, 2, -1.5707963267948966, -3.141592653589793, 0.5, 0.8232787691260659, 0.21042481641773064, 0.25378105882352936, 0.000648084977482323, -0.007518255527237134, 0.9999302406011185, -0.009086789921561894], [0, -1.5707963267948966, 2, -1.5707963267948966, -3.141592653589793, 0.5, 0.822255898222331, 0.04135489357405592, 0.25374954901960783, -0.003596721462689725, -0.004919289131052951, 0.9997276271867958, -0.022528551409800462], [0, -1.5707963267948966, 2, -1.5707963267948966, -3.141592653589793, 0.5, 0.820149912691993, -0.12245399632326899, 0.2522424901960784, 0.00021183661483716693, -0.013426393636156594, 0.9998216893511835, -0.013276919150201618]]
from agimus_demos import InStatePlanner
inStatePlanner = InStatePlanner(ps, graph)
inStatePlanner.maxIterPathPlanning = 66
inStatePlanner.optimizerTypes = ['SimpleTimeParameterization']
#inStatePlanner.plannerType  = 'M-RRT'

###
## generate target
##def inStatePlanner(go,q_final):
edge = ['approach-part','grasp-part','take-part-up','take-part-away']

q_final = [0, -1.5707963267948966, 2.0, -1.5707963267948966, 0, 0.5, 0.9349991154141712, 0.8, 0.05, 0.007663720375115889, -0.005089128671218016, 0.9995081203537567, -0.02998141935065666,0,0,0,0,0,0,1]
###The function uses inStatePlanner to generate path followes given state and edge,at last get concatenated path PID 0
def instatePath(q_start,q_final,a=0):
    q_goal = []
    edge = ['approach-part','grasp-part','take-part-up','take-part-away']
    res = False    
    res2 = [False]
    while not (res and res2[0]):
        q = robot.shootRandomConfig ()
        res,q2,err = graph.applyNodeConstraints ('free', q_start)
        res2 = robot.isConfigValid (q2)
    q_goal.append(q2)
    for i in range(len(edge)) :
        path = False
        res =  False
        while not path:
            
            print('new start',i)
            q1 = robot.shootRandomConfig ()
            res,q1,err = graph.generateTargetConfig (edge[i], q_goal[i],q1)
            print('for generate',res)
            if not res: continue
            """ res = robot.isConfigValid (q1)
            print('for valid q1',res)
            if not res: continue """
            inStatePlanner.setEdge(edge[i])
            pv = inStatePlanner.cedge.getPathValidation()
            #pv = inStatePlanner.cproblem.getPathValidation()
            res, msg = pv.validateConfiguration(q_goal[i])
            print('for validate q_goal',res,msg)
            if not res: continue
            res, msg = pv.validateConfiguration(q1)
            #print('q1',res,msg)
            if not res:continue
            print(q1)
            v(q1)
            try:
                print(q_goal[i],q1)
                Path = inStatePlanner.computePath(q_goal[i],[q1])
            except Exception as e:
                print('no path this time',e)
            else:
                #Path =  inStatePlanner.optimizePath(Path)
                q_goal.append(q1) 
                #Path = inStatePlanner.computePath(q_goal[i],[q_goal[i+1]])
                print(q_goal)
                path = True
                vector  = Path.asVector()
                pid = ps.client.basic.problem.addPath(vector)
                print('find path for edge %d'%(i))
        if i ==3:
            """ res = False
            res2 = [False]
            while not (res and res2[0]):
                q = robot.shootRandomConfig ()
                res,q1,err = graph.applyNodeConstraints ('vertical', q_final)
                res2 = robot.isConfigValid (q1) """ 
            res = False
            res2 = [False]
            while not (res and res2[0]):
                q = robot.shootRandomConfig ()
                res,q2,err = graph.applyNodeConstraints ('free', q_final)
                res2 = robot.isConfigValid (q2)
            q_goal.append(q2)   
            for j in range(len(edge)) :
                path = False
                res =  False
                while not path:
                    #if j != 3: not robust
            
                    print('new start for second part',j)
                    q1 = robot.shootRandomConfig ()
                    res,q1,err = graph.generateTargetConfig (edge[j], q_goal[j+5],q1)
                    if not res: continue
                    res = robot.isConfigValid (q1)
                    if not res: continue
                    inStatePlanner.setEdge(edge[j])
                    pv = inStatePlanner.cproblem.getPathValidation()
                    res, msg = pv.validateConfiguration(q_goal[j+5])
                    if not res: continue
                    res, msg = pv.validateConfiguration(q1)
                    if not res:continue
                    if j ==3:
                        q1 = q_goal[4]
                    print(q1)
                    v(q1)
                    try:
                        Path = inStatePlanner.computePath(q_goal[j+5],[q1])
                    except Exception as e:
                        print('no path this time',e)
                    else: 
                        #Path = inStatePlanner.optimizePath(Path)
                        q_goal.append(q1) 
                #Path = inStatePlanner.computePath(q_goal[i],[q_goal[i+1]])
                        print(q_goal)
                        path = True
                        Path = Path.reverse()
                        vector  = Path.asVector()
                        pid = ps.client.basic.problem.addPath(vector)
                        print('find path for edge %d'%(j))
            #inStatePlanner.setEdge('transfer')
            #Path = inStatePlanner.computePath(q_goal[4],[q_goal[9]])
                            #Path = inStatePlanner.optimizePath(Path)
            #vector  = Path.asVector()
            #pid = ps.client.basic.problem.addPath(vector)
            list = [i for i in range(1+a*8,a*8+8)]
            list[3:8] =list[7:2:-1]
            for i in list:
                ps.client.basic.problem.concatenatePath(a*8,i)
   
        #ps.addPathOptimizer('SimpleTimeParameterization')
        #ps.optimizePath(0)



#### To test Security margin
ini = [0, -1.5707963267948966, 2.796017461694916, -1.5707963267948966, -3.141592653589793, 0.5, 0.9349991154141712, 0.039512694934721385, 0.2544894705882353, 0.007663720375115889, -0.005089128671218016, 0.9995081203537567, -0.02998141935065666, 0, 0, 0.1, 0, 0, 0, 1]

res,res2 = False,False
while not (res and res2[0]):
    q = robot.shootRandomConfig ()
    res,q1,err = graph.applyNodeConstraints ('free', ini)
    res2 = robot.isConfigValid (q1)

res,res2 = False,False
while not (res and res2[0]):
    q = robot.shootRandomConfig ()
    res,q2,err = graph.generateTargetConfig ('approach-part', q1, q)
    res2 = robot.isConfigValid (q2)

res,res2 = False,False
while not res:
    q = robot.shootRandomConfig ()
    res,q3,err = graph.generateTargetConfig ('grasp-part', q2, q)
inStatePlanner.setEdge(edge[1])
pv = inStatePlanner.cedge.getPathValidation()
pv.validateConfiguration(q3)
###Test for muitiple object

goal = [go[1],go[3]]
q_final = [0, -1.5707963267948966, 2.796017461694916, -1.5707963267948966, -3.141592653589793, 0.5, 1.0432209178713296, 0.8, 0.03, 0.010857436013281468, -0.007109644686263878, 0.99955548479711, -0.026840302674650103, 0, 0, 0.1, 0, 0, 0, 1]

for i in range(len(goal)):
    instatePath(goal[i],q_final,a = i)
Nb_path = ps.numberPaths()
for i in range(1,Nb_path):
    if i%8== 0:
        ps.concatenatePath(0,i)  