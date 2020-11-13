from grasp_ball import q_init, q_goal, robot, ps, graph, vf

success = False
trial = 0
res = False
while not success:
    paths = list ()
    print ("trial {0} res {1}".format (trial,res)); trial += 1
    q = robot.shootRandomConfig ()
    res, q1, err = graph.generateTargetConfig ('approach-ball', q_init, q)
    if not res: continue
    res, msg = robot.isConfigValid (q1)
    if not res: continue
    res, pid, msg = ps.directPath (q_init, q1, True)
    paths.append (pid)
    if not res: continue
    success = True
print("approach ok")
q_1 = q1
success = False
trial = 0
while not success and trial < 300:
    print ("trial {0} res {1}".format (trial,res)); trial += 1
    q = robot.shootRandomConfig ()
    res, q2, err = graph.generateTargetConfig ('grasp-ball', q1, q)
    if not res: continue
    res, msg = robot.isConfigValid (q2)
    if not res: continue
    res, pid, msg = ps.directPath (q1, q2, True)
    paths.append (pid)
    if not res:
        q1 = q2
        continue
    success = True
q_2 = q2
success = False
trial = 0
while not success and trial < 300:
    print ("trial {0} res {1}".format (trial,res)); trial += 1
    q = robot.shootRandomConfig ()
    res, q1, err = graph.generateTargetConfig ('take-ball-up', q2, q)
    if not res: continue
    res, msg = robot.isConfigValid (q1)
    if not res: continue
    res, pid, msg = ps.directPath (q2, q1, True)
    paths.append (pid)
    if not res:
        q2 = q1
        continue
    success = True
print("grasp ok")
