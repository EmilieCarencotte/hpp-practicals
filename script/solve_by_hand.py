from grasp_ball import q_init, q_goal, robot, ps, graph, vf, PathPlayer

success = False
trial = 0
res = False
res1 = False
res2 = False
while not success and trial < 3000:
    print ("trial {0} res {1},{2},{3}".format (trial,res,res1,res2)); trial += 1
    res = False
    res1 = False
    while not res:
        paths = list ()
        q = robot.shootRandomConfig ()
        res, q1, err = graph.generateTargetConfig ('approach-ball', q_init, q)
        if not res: continue
        res, msg = robot.isConfigValid (q1)
        if not res: continue
        res, pid, msg = ps.directPath (q_init, q1, True)
        paths.append (pid)
        if not res: continue
    res = False
    
    while not res:
        q = robot.shootRandomConfig ()
        res, q2, err = graph.generateTargetConfig ('grasp-ball', q1, q)
        if not res: continue
        res, msg = robot.isConfigValid (q2)
        if not res: continue
        res, pid, msg = ps.directPath (q1, q2, True)
        if not res: continue
        paths.append(pid)
    res = False
    while not res:
        q = robot.shootRandomConfig ()
        res, q3, err = graph.generateTargetConfig ('take-ball-up', q2, q)
        if not res: continue
        res, msg = robot.isConfigValid (q3)
        if not res: continue
        res, pid, msg = ps.directPath (q2, q3, True)
        if not res:continue
        paths.append (pid)
    res = False

    while not res:
        q = robot.shootRandomConfig ()
        res, q31, err = graph.generateTargetConfig ('move-ball', q3, q)
        if not res: continue
        res, msg = robot.isConfigValid (q31)
        if not res: continue
        res, pid, msg = ps.directPath(q3, q31, True)
        paths.append(pid)

    while not res1:
        paths2 = list ()
        q = robot.shootRandomConfig ()
        res1, q6, err = graph.generateTargetConfig ('approach-ball', q_goal, q)
        if not res1: continue
        res1, msg = robot.isConfigValid (q6)
        if not res1: continue
        res1, pid, msg = ps.directPath (q6, q_goal, True)
        if not res1: continue
        paths2.append(pid)
    res1 = False

    while not res1:
        q = robot.shootRandomConfig ()
        res1, q5, err = graph.generateTargetConfig ('grasp-ball', q6, q)
        if not res1: continue
        res1, msg = robot.isConfigValid (q5)
        if not res1: continue
        res1, pid, msg = ps.directPath (q5, q6, True)
        if not res1: continue
        paths2.append(pid)
    res1 = False

    while not res1:
        q = robot.shootRandomConfig ()
        res1, q4, err = graph.generateTargetConfig ('take-ball-up', q5, q)
        if not res1: continue
        res1, msg = robot.isConfigValid (q4)
        if not res1: continue
        res1, pid, msg = ps.directPath (q4, q5, True)
        if not res1: continue
        paths2.append(pid)
    res1 = False
    
    while not res1:
        q = robot.shootRandomConfig ()
        res1, q41, err = graph.generateTargetConfig ('move-ball', q4, q)
        if not res1: continue
        res1, msg = robot.isConfigValid (q41)
        if not res1: continue
        res1, pid, msg = ps.directPath (q41, q4, True)
        if not res1: continue
        paths2.append(pid)

    res2,pid2,msg = ps.directPath(q31,q41,True)
    res1,_,_ = ps.directPath(q31,q41,False)
    if not (res1 and res2):continue
    paths.append(pid2)
    success = True

def generatePathOnEdge(edge,q_start,is_start):
    sys.stdout.write("research for edge {} linked to {}...".format(edge,graph.getNodesConnectedByEdge(edge)[1])) # I use stoud to use carrier return
    sys.stdout.flush()
    pid = -1
    res1 = False
    res2 = False
    MAXTEST = 3000

    for i in range (1,MAXTEST):
        sys.stdout.write("\rresearch for edge {} linked to {} essai {}/{}".format(edge,graph.getNodesConnectedByEdge(edge)[1],i,MAXTEST))
        sys.stdout.flush()
        q = robot.shootRandomConfig ()
        res1,qi,_ = graph.generateTargetConfig (edge, q_start, q)
        if res1 and robot.isConfigValid (qi):
            if is_start:
                res2, pid, msg = ps.directPath(q_start, qi, True)
            else:
                res2, pid, msg = ps.directPath(qi, q_start, True)
            if res2 :         # if we have a direct path to a random valid position corresponding to the target config, let's continue path-planning
                break;            
    
    sys.stdout.write("\rresearch for edge {} linked to {} {} at essai {}/{}\n".format(edge,graph.getNodesConnectedByEdge(edge)[1],"success" if (res1 and res2) else "failed",i,MAXTEST))
    return res1 and res2 ,qi,pid

while not success:
    if trial > 100: break;
    paths_qinit = list()
    paths_qgoal = list()
    print ("trial {}".format(trial)); trial += 1
    
    # take the ball
    print ("take the ball")
    q_start = q_init
    for edge in list_of_edges:
        res,qi1,pid = generatePathOnEdge(edge,q_start,true)
        if not res: break
        paths_qinit.append(pid)
        q_start = qi1
    if not res: continue
        
    # put the ball in q_goal
    print ("put the ball in q_goal")
    q_start = q_goal
    for edge in list_of_edges:
        res,qi2,pid = generatePathOnEdge(edge,q_start,false)
        if not res: break
        paths_qgoal.append(pid)
        q_start = qi2
    if not res: continue
        
    #try to link the two paths
    res,pid,_ = ps.directPath(qi1,qi2,True)
    if not res: continue
    paths_qinit.append(pid)
    success = True
        
paths_qgoal.reverse()
paths = paths_qinit + paths_qgoal

print("solve done, let's display the path obtained")

v = vf.createViewer()
pp = PathPlayer(v)

for p in paths:
    pp(p)
