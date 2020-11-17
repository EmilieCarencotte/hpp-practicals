from grasp_ball import q_init, q_goal, robot, ps, graph, vf, PathPlayer

#success = False
#trial = 0
#res = False
#while not success:
#    paths = list ()
#    print ("trial {0} res {1}".format (trial,res)); trial += 1
#    q = robot.shootRandomConfig ()
#    res, q1, err = graph.generateTargetConfig ('approach-ball', q_init, q)
#    if not res: continue
#    res, msg = robot.isConfigValid (q1)
#    if not res: continue
#    res, pid, msg = ps.directPath (q_init, q1, True)
#    paths.append (pid)
#    if not res: continue
#    success = True
#print("approach ok")

#success = False
#trial = 0
#while not success and trial < 300:
#    print ("trial {0} res {1}".format (trial,res)); trial += 1
#    q = robot.shootRandomConfig ()
#    res, q2, err = graph.generateTargetConfig ('grasp-ball', q1, q)
#    if not res: continue
#    res, msg = robot.isConfigValid (q2)
#    if not res: continue
#    res, pid, msg = ps.directPath (q1, q2, True)
#    if not res: continue
#    paths.append(pid)
#    success = True

#success = False
#trial = 0
#while not success and trial < 300:
#    print ("trial {0} res {1}".format (trial,res)); trial += 1
#    q = robot.shootRandomConfig ()
#    res, q3, err = graph.generateTargetConfig ('take-ball-up', q2, q)
#    if not res: continue
#    res, msg = robot.isConfigValid (q3)
#    if not res: continue
#    res, pid, msg = ps.directPath (q2, q3, True)
#    if not res:continue
#    paths.append (pid)
#    success = True
res = False
success = False
trial = 0
while not success and trial < 300:
    paths = list()
    print ("trial {0} res {1}".format (trial,res)); trial += 1
    q = robot.shootRandomConfig ()
    res, q4, err = graph.generateTargetConfig ('approach-ball', q_goal, q)
    if not res: continue
    res, msg = robot.isConfigValid (q4)
    if not res: continue
    res, pid, msg = ps.directPath (q4, q_goal, True)
    #if not res:continue
    paths.append (pid)
    success = True
print("grasp ok")
