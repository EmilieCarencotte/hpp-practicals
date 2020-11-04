class MotionPlanner:
  def __init__ (self, robot, ps):
    self.robot = robot
    self.ps = ps

  def getNewConfig (self, q_rand, connectedComponentId, list):
    q_near,_ = self.ps.getNearestConfig(q_rand, connectedComponentId);
    _,new_path_id,_ = self.ps.directPath(q_near, q_rand, True);
    length = self.ps.pathLength(new_path_id);
    q_new = self.ps.configAtParam(new_path_id, length);
    self.ps.addConfigToRoadmap(q_new);
    self.ps.addEdgeToRoadmap(q_near, q_new, new_path_id, True);
    list.append(q_new);

  def connectNodes (self, config_list, new_config):
    for q in config_list:
      is_valid, path_id, _ = self.ps.directPath(new_config, q, True);
      if is_valid:
        self.ps.addEdgeToRoadmap(q, new_config, path_id, True);
        return;
      
    

  def solveBiRRT (self, maxIter = float("inf")):
    print ("Method solveBiRRT is not implemented yet")
    self.ps.prepareSolveStepByStep ()
    finished = False

    # In the framework of the course, we restrict ourselves to 2 connected components.
    nbCC = self.ps.numberConnectedComponents ()
    if nbCC != 2:
      raise Exception ("There should be 2 connected components.")

    iter = 0
    beginConfigs = list()
    beginConfigs.append(self.ps.getInitialConfig())
    endConfigs = list()
    endConfigs.append(self.ps.getGoalConfigs()[0])
    
    while True:
      #### RRT begin

      q_rand = self.robot.shootRandomConfig()
      self.getNewConfig(q_rand, 0, beginConfigs)
      self.getNewConfig(q_rand, 1, endConfigs)

      ## Try connecting the new nodes together
      for q in beginConfigs:
        is_valid, path_id, _ = self.ps.directPath(q, endConfigs[-1], True);
        if is_valid:
          self.ps.addEdgeToRoadmap(q, endConfigs[-1], path_id, True);
      for q in endConfigs:
        is_valid, path_id, _ = self.ps.directPath(beginConfigs[-1], q, True);
        if is_valid:
          self.ps.addEdgeToRoadmap(beginConfigs[-1], q, path_id, True);
##      self.connectNodes(endConfigs, beginConfigs[-1])
##      self.connectNodes(beginConfigs, endConfigs[-1])

      
      #### RRT end
      ## Check if the problem is solved.
      nbCC = self.ps.numberConnectedComponents ()
      if nbCC == 1:
        # Problem solved
        finished = True
        break
      iter = iter + 1
      if iter > maxIter:
        break
    if finished:
        self.ps.finishSolveStepByStep ()
        return self.ps.numberPaths () - 1

  def solvePRM (self):
    self.ps.prepareSolveStepByStep ()
    #### PRM begin
    #### PRM end
    self.ps.finishSolveStepByStep ()
