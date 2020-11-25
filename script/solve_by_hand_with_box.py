from grasp_ball_in_box import q_init, q_goal, robot, ps, graph, vf, PathPlayer
import sys

success = False
trial = 0
res = False
res1 = False
res2 = False

while not success:
    trial = 0
    print ("res {1},{2},{3}".format (trial,res,res1,res2)); trial += 1
    res = False
    res1 = False
    while not res and trial < 3000:
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
    
    while not res and trial < 3000:
        print(trial); trial += 1
        q = robot.shootRandomConfig ()
        res, q2, err = graph.generateTargetConfig ('grasp-ball', q1, q)
        if not res: continue
        res, msg = robot.isConfigValid (q2)
        if not res: continue
        res, pid, msg = ps.directPath (q1, q2, True)
        if not res: continue
        paths.append(pid)
    res = False
    while not res and trial < 3000:
        q = robot.shootRandomConfig ()
        res, q3, err = graph.generateTargetConfig ('take-ball-up', q2, q)
        if not res: continue
        res, msg = robot.isConfigValid (q3)
        if not res: continue
        res, pid, msg = ps.directPath (q2, q3, True)
        if not res:continue
        paths.append (pid)
    res = False

    while not res and trial < 3000:
        q = robot.shootRandomConfig ()
        res, q31, err = graph.generateTargetConfig ('move-ball', q3, q)
        if not res: continue
        res, msg = robot.isConfigValid (q31)
        if not res: continue
        res, pid, msg = ps.directPath(q3, q31, True)
        if not res:continue
        paths.append(pid)

    while not res1 and trial < 3000:
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

    while not res1 and trial < 3000:
        q = robot.shootRandomConfig ()
        res1, q5, err = graph.generateTargetConfig ('grasp-ball', q6, q)
        if not res1: continue
        res1, msg = robot.isConfigValid (q5)
        if not res1: continue
        res1, pid, msg = ps.directPath (q5, q6, True)
        if not res1: continue
        paths2.append(pid)
    res1 = False

    while not res1 and trial < 3000:
        q = robot.shootRandomConfig ()
        res1, q4, err = graph.generateTargetConfig ('take-ball-up', q5, q)
        if not res1: continue
        res1, msg = robot.isConfigValid (q4)
        if not res1: continue
        res1, pid, msg = ps.directPath (q4, q5, True)
        if not res1: continue
        paths2.append(pid)
    res1 = False
    
    while not res1 and trial < 3000:
        q = robot.shootRandomConfig ()
        res1, q41, err = graph.generateTargetConfig ('move-ball', q4, q)
        if not res1: continue
        res1, msg = robot.isConfigValid (q41)
        if not res1: continue
        res1, pid, msg = ps.directPath (q41, q4, True)
        if not res1: continue
        paths2.append(pid)

    if trial < 3000:
        res2,pid2,msg = ps.directPath(q31,q41,True)
        res1,_,_ = ps.directPath(q31,q41,False)
        if not (res1 and res2):continue
        paths.append(pid2)
        success = True

paths2.reverse()
paths_final = paths + paths2

print("grasp ok")

v = vf.createViewer()
pp = PathPlayer(v)

for p in paths_final:
    pp(p)
