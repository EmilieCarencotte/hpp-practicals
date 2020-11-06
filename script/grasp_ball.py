from math import sqrt
from hpp import Transform
from hpp.corbaserver.manipulation import ConstraintGraph, Constraints
from hpp.corbaserver import Client
Client ().problem.resetProblem ()
from manipulation import robot, vf, ps, Ground, Box, Pokeball, PathPlayer, gripperName, ballName

vf.loadEnvironmentModel (Ground, 'ground')
vf.loadObjectModel (Pokeball, 'pokeball')
robot.setJointBounds ('pokeball/root_joint', [-.4,.4,-.4,.4,-.1,2.,
                                              -1.0001, 1.0001,-1.0001, 1.0001,
                                              -1.0001, 1.0001,-1.0001, 1.0001,])


q1 = [0, -1.57, 1.57, 0, 0, 0, .3, 0, 0.025, 0, 0, 0, 1]

## Create constraint graph
graph = ConstraintGraph (robot, 'graph')

## Create constraint of relative position of the ball in the gripper when ball
## is grasped
ballInGripper = [0, .137, 0, 0.5, 0.5, -0.5, 0.5]
ps.createTransformationConstraint ('grasp', gripperName, ballName,
                                   ballInGripper, 6*[True,])

## Create nodes and edges
#  Warning the order of the nodes is important. When checking in which node
#  a configuration lies, node constraints will be checked in the order of node
#  creation.
graph.createNode (['grasp', 'ball-above-ground', 'grasp-placement', 'gripper-above-ball', 'placement'])
graph.createEdge ('placement', 'placement', 'transit', 1, 'placement')
graph.createEdge ('grasp', 'grasp', 'transfer', 1, 'grasp')
#graph.createEdge ('placement', 'grasp', 'grasp-ball', 1, 'placement')
#graph.createEdge ('grasp', 'placement', 'release-ball', 1, 'grasp')

graph.createEdge ('placement', 'gripper-above-ball', 'approach-ball', 1, 'placement')
graph.createEdge ('gripper-above-ball', 'placement', 'move-gripper-away', 1, 'gripper-above-ball')
graph.createEdge ('gripper-above-ball', 'grasp-placement', 'grasp-ball', 1, 'gripper-above-ball')
graph.createEdge ('grasp-placement', 'gripper-above-ball', 'move-gripper-up', 1, 'grasp-placement')
graph.createEdge ('grasp-placement', 'ball-above-ground', 'take-ball-up', 1, 'grasp-placement')
graph.createEdge ('ball-above-ground', 'grasp-placement', 'put-ball-down', 1, 'ball-above-ground')
graph.createEdge('ball-above-ground', 'grasp', 'take-ball-away', 1, 'ball-above-ground')
graph.createEdge('grasp', 'ball-above-ground', 'approach-ground', 1, 'grasp')

## Create transformation constraint : ball is in horizontal plane with free
## rotation around z
ps.createTransformationConstraint ('placementBallOnGround', '', ballName,
                                   [0,0,0.025,0, 0, 0, 1],
                                   [False, False, True, True, True, False,])
ps.createTransformationConstraint ('placementBallAboveGround', '', ballName,
                                   [0, 0, 0.325, 0, 0, 0, 1],
                                   [False, False, True, True, True, False,])
#  Create complement constraint
ps.createTransformationConstraint ('placement/complement', '', ballName,
                                   [0,0,0.025,0, 0, 0, 1],
                                   [True, True, False, False, False, True,])

ballUnderGripper = [0, .337, 0, 0.5, 0.5, -0.5, 0.5]
ps.createTransformationConstraint ('gripper-above-ball', gripperName, ballName,
                                   ballUnderGripper,
                                   [True, False, True, True, True, True])

ps.setConstantRightHandSide ('placementBallOnGround', True)
ps.setConstantRightHandSide ('placementBallAboveGround', True)
ps.setConstantRightHandSide ('placement/complement', False)
ps.setConstantRightHandSide ('gripper-above-ball', False)

## Set constraints of nodes and edges
graph.addConstraints (node='placement', constraints = \
                      Constraints (numConstraints = ['placementBallOnGround'],))
graph.addConstraints (node='grasp',
                      constraints = Constraints (numConstraints = ['grasp']))
graph.addConstraints (node='gripper-above-ball', constraints = \
                      Constraints (numConstraints = ['gripper-above-ball', 'placementBallOnGround']))
graph.addConstraints (node='grasp-placement', constraints = \
                      Constraints (numConstraints = ['grasp', 'placementBallOnGround']))
graph.addConstraints (node='ball-above-ground', constraints = \
                      Constraints (numConstraints = ['grasp', 'placementBallAboveGround']))

graph.addConstraints (edge='transit', constraints = \
                      Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='approach-ball', constraints = \
                      Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='move-gripper-away', constraints = \
                      Constraints (numConstraints = ['placement/complement']))

#graph.addConstraints (edge='grasp-ball', constraints = \
#                      Constraints (numConstraints = ['placement/complement']))


# These edges are in node 'grasp'
graph.addConstraints (edge='transfer',     constraints = Constraints ())
graph.addConstraints (edge='move-gripper-up',     constraints = Constraints ())
graph.addConstraints (edge='grasp-ball',     constraints = Constraints ())
graph.addConstraints (edge='take-ball-up',     constraints = Constraints ())
graph.addConstraints (edge='put-ball-down',     constraints = Constraints ())
graph.addConstraints (edge='take-ball-away',     constraints = Constraints ())
graph.addConstraints (edge='approach-ground',     constraints = Constraints ())

#graph.addConstraints (edge='release-ball', constraints = Constraints ())
ps.selectPathValidation ("Dichotomy", 0)
ps.selectPathProjector ("Progressive", 0.1)
graph.initialize ()

## Project initial configuration on state 'placement'
res, q_init, error = graph.applyNodeConstraints ('placement', q1)
q2 = q1 [::]
q2 [7] = .5
q2 [6] = .7

## Project goal configuration on state 'placement'
res, q_goal, error = graph.applyNodeConstraints ('placement', q2)

#v = vf.createViewer()
#q_t = q1[::]
#q_t[7] = 1
#b, q_test, e = graph.generateTargetConfig('transfer', q1, q_t)
#v(q_init)

## Define manipulation planning problem
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

# v = vf.createViewer ()
# pp = PathPlayer (v)
# v (q1)

## Build relative position of the ball with respect to the gripper
for i in range (100):
  q = robot.shootRandomConfig ()
  res,q3,err = graph.generateTargetConfig ('grasp-ball', q_init, q)
  if res and robot.isConfigValid (q3): break;

if res:
  robot.setCurrentConfig (q3)
  gripperPose = Transform (robot.getJointPosition (gripperName))
  ballPose = Transform (robot.getJointPosition (ballName))
  gripperGraspsBall = gripperPose.inverse () * ballPose
  gripperAboveBall = Transform (gripperGraspsBall)
  gripperAboveBall.translation [2] += .1
