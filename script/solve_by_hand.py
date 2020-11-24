from grasp_ball import q_init, q_goal, robot, ps, graph, vf, PathPlayer

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
    if not res: continue
    paths.append(pid)
    success = True
print("grasp-ball ok")

success = False
trial = 0
while not success and trial < 300:
    print ("trial {0} res {1}".format (trial,res)); trial += 1
    q = robot.shootRandomConfig ()
    res, q3, err = graph.generateTargetConfig ('take-ball-up', q2, q)
    if not res: continue
    res, msg = robot.isConfigValid (q3)
    if not res: continue
    res, pid, msg = ps.directPath (q2, q3, True)
    if not res:continue
    paths.append (pid)
    success = True
print("ball up ok")

success = False
trial = 0
while not success and trial < 300:
    print ("trial {0} res {1}".format (trial,res)); trial += 1
    q = robot.shootRandomConfig ()
    res, q31, err = graph.generateTargetConfig ('move-ball', q3, q)
    if not res: continue
    res, msg = robot.isConfigValid (q31)
    if not res: continue
    res, pid, msg = ps.directPath (q3, q31, True)
    if not res:continue
    paths.append (pid)
    success = True
print("ball up ok")

success = False
trial = 0
res = False
while not success:
    paths2 = list ()
    print ("trial {0} res {1}".format (trial,res)); trial += 1
    q = robot.shootRandomConfig ()
    res, q6, err = graph.generateTargetConfig ('approach-ball', q_goal, q)
    if not res: continue
    res, msg = robot.isConfigValid (q6)
    if not res: continue
    res, pid, msg = ps.directPath (q6, q_goal, True)
    paths2.append (pid)
    if not res: continue
    success = True
print("approach ok")

success = False
trial = 0
while not success and trial < 300:
    print ("trial {0} res {1}".format (trial,res)); trial += 1
    q = robot.shootRandomConfig ()
    res, q5, err = graph.generateTargetConfig ('grasp-ball', q6, q)
    if not res: continue
    res, msg = robot.isConfigValid (q5)
    if not res: continue
    res, pid, msg = ps.directPath (q5, q6, True)
    if not res: continue
    paths2.append(pid)
    success = True
print("grasp-ball ok")

success = False
trial = 0
while not success and trial < 300:
    print ("trial {0} res {1}".format (trial,res)); trial += 1
    q = robot.shootRandomConfig ()
    res, q4, err = graph.generateTargetConfig ('take-ball-up', q5, q)
    if not res: continue
    res, msg = robot.isConfigValid (q4)
    if not res: continue
    res, pid, msg = ps.directPath (q4, q5, True)
    if not res:continue
    paths2.append (pid)
    success = True
print("ball up ok")

paths2.reverse()
res, pid, msg = ps.directPath(q31,q4,False)
paths.append(pid)
paths_final = paths + paths2

print("grasp ok")
