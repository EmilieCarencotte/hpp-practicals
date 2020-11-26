from math import sqrt
from hpp import Transform
from hpp.corbaserver.manipulation import ConstraintGraph, Constraints
from hpp.corbaserver import Client
Client ().problem.resetProblem ()
from manipulation import robot, ps, vf, Ground, Box, Pokeball, PathPlayer, gripperName, ballName
import sys

vf.loadEnvironmentModel (Ground, 'ground')
vf.loadEnvironmentModel (Box, 'box')
vf.moveObstacle ('box/base_link_0', [0.3+0.04, 0, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_1', [0.3-0.04, 0, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_2', [0.3, 0.04, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_3', [0.3, -0.04, 0.04, 0, 0, 0, 1])

vf.loadObjectModel (Pokeball, 'pokeball')
robot.setJointBounds ('pokeball/root_joint', [-.4,.4,-.4,.4,-.1,1.,
                                              -1.0001, 1.0001,-1.0001, 1.0001,
                                              -1.0001, 1.0001,-1.0001, 1.0001,])



q1 = [0, -1.57, 1.57, 0, 0, 0, .3, 0, 0.025, 0, 0, 0, 1]

## Create constraint graph
graph = ConstraintGraph (robot, 'graph')

## Create constraint of relative position of the ball in the gripper when ball
## is grasped
ballInGripper    = [0, .137, 0, 0.5, 0.5, -0.5, 0.5]
ballUnderGripper = [0, .237, 0, 0.5, 0.5, -0.5, 0.5]

## Create nodes and edges
#  Warning the order of the nodes is important. When checking in which node
#  a configuration lies, node constraints will be checked in the order of node
#  creation.
graph.createNode (['ball-above-ground', 'grasp-placement', 'grasp', 'gripper-above-ball', 'placement'])

graph.createEdge ('placement'         , 'placement'         , 'transit'          , 1, 'placement')
graph.createEdge ('placement'         , 'gripper-above-ball', 'approach-ball'    , 1, 'placement')
graph.createEdge ('gripper-above-ball', 'placement'         , 'move-gripper-away', 1, 'placement')
graph.createEdge ('gripper-above-ball', 'grasp-placement'   , 'grasp-ball'       , 1, 'placement')
graph.createEdge ('grasp-placement'   , 'gripper-above-ball', 'move-gripper-up'  , 1, 'placement')
graph.createEdge ('grasp-placement'   , 'ball-above-ground' , 'take-ball-up'     , 1, 'grasp')
graph.createEdge ('ball-above-ground' , 'grasp-placement'   , 'put-ball-down'    , 1, 'grasp')
graph.createEdge ('ball-above-ground' , 'ball-above-ground' , 'move-ball'        , 1, 'grasp')
graph.createEdge ('ball-above-ground' , 'grasp'             , 'take-ball-away'   , 1, 'grasp')
graph.createEdge ('grasp'             , 'ball-above-ground' , 'approach-ground'  , 1, 'grasp')
graph.createEdge ('grasp'             , 'grasp'             , 'transfer'         , 1, 'grasp')

ps.createTransformationConstraint ('grasp', gripperName, ballName,
                                   ballInGripper, 6*[True,])
ps.createTransformationConstraint ('placementAboveBall', gripperName, ballName,
                                   ballUnderGripper, 6*[True,])

## Create transformation constraint : ball is in horizontal plane with free
# rotation around z
ps.createTransformationConstraint ('placementBallOnGround', '', ballName,
                                   [0, 0, 0.025, 0, 0, 0, 1],
                                   [False, False, True, True, True, False,])

ps.createTransformationConstraint ('placementBallAboveGround', '', ballName,
                                   [0, 0, 0.225, 0, 0, 0, 1],
                                   [False, False, True, True, True, False,])

#  Create complement constraint
ps.createTransformationConstraint ('placement/complement', '', ballName,
                                   [0, 0, 0.025, 0, 0, 0, 1],
                                   [True, True, False, False, False, True,])

ps.setConstantRightHandSide ('placementBallOnGround', True)
ps.setConstantRightHandSide ('placementBallAboveGround', True)

ps.setConstantRightHandSide ('placement/complement', False)
ps.setConstantRightHandSide ('placementAboveBall', True)

## Set constraints of nodes and edges
graph.addConstraints (node='placement'         , constraints = Constraints (numConstraints = ['placementBallOnGround'],))
graph.addConstraints (node='gripper-above-ball', constraints = Constraints (numConstraints = ['placementBallOnGround', 'placementAboveBall']))
graph.addConstraints (node='grasp-placement'   , constraints = Constraints (numConstraints = ['grasp', 'placementBallOnGround']))
graph.addConstraints (node='ball-above-ground' , constraints = Constraints (numConstraints = ['grasp', 'placementBallAboveGround']))
graph.addConstraints (node='grasp'             , constraints = Constraints (numConstraints = ['grasp']))

graph.addConstraints (edge='transit'          , constraints = Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='approach-ball'    , constraints = Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='move-gripper-away', constraints = Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='move-gripper-up'  , constraints = Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='grasp-ball'       , constraints = Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='take-ball-up'     , constraints = Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='put-ball-down'    , constraints = Constraints (numConstraints = ['placement/complement']))

# These edges are in node 'grasp'
graph.addConstraints (edge='transfer'      , constraints = Constraints ())
graph.addConstraints (edge='take-ball-away', constraints = Constraints ())
graph.addConstraints (edge='move-ball'     , constraints = Constraints ())

#graph.addConstraints (edge='release-ball', constraints = Constraints ())
ps.selectPathValidation ("Dichotomy", 0)
ps.selectPathProjector ("Progressive", 0.1)
graph.initialize ()

## Project initial configuration on state 'placement'
res, q_init, error = graph.applyNodeConstraints ('placement', q1)
q2 = q1 [::]
q2 [7] = .2

## Project goal configuration on state 'placement'
res, q_goal, error = graph.applyNodeConstraints ('placement', q2)

## Define manipulation planning problem
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

v = vf.createViewer ()
pp = PathPlayer (v)
# v (q1)

## Build relative position of the ball with respect to the gripper
for i in range (100):
  q = robot.shootRandomConfig ()
  res1,q3,err = graph.generateTargetConfig ('approach-ball', q_init, q)
  if res1 and robot.isConfigValid (q3): break;
    
if res:
  robot.setCurrentConfig (q3)
  gripperPose = Transform (robot.getJointPosition (gripperName))
  ballPose = Transform (robot.getJointPosition (ballName))
  gripperGraspsBall = gripperPose.inverse () * ballPose
  gripperAboveBall = Transform (gripperGraspsBall)
  gripperAboveBall.translation [2] += .1
  
def solve():
  print("Solver started")
  ps.solve()
  print("Solution found, let's display it")
  pp(0)

if('grasp_ball_in_box.py' in sys.argv[0]):
  print("execute solve() to resolve and display the path")
