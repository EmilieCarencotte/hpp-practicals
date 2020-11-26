from grasp_ball_in_box import q_init, q_goal, robot, ps, graph, vf, PathPlayer
import sys

import sys

success = False
trial = 0
list_of_edges = ['approach-ball', 'grasp-ball', 'take-ball-up']
    
def generatePathOnEdge(edge,q_start,is_start):
    sys.stdout.write("edge {}".format(edge)) # I use stoud to use carrier return
    sys.stdout.flush()
    pid = -1
    res1 = False
    res2 = False
    MAXTEST = 300
    for i in range (1,MAXTEST):
        sys.stdout.write("\redge {} essai {}/{}".format(edge,i,MAXTEST))
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
    
    sys.stdout.write("\redge {} {} at essai {}/{}\n".format(edge,"success" if (res1 and res2) else "failed",i,MAXTEST))
    return res1 and res2 ,qi, pid

while not success:
    if trial > 100: break;
    paths_qinit = list()
    paths_qgoal = list()
    print ("trial {}".format(trial)); trial += 1
    
    # take the ball
    print ("take the ball")
    q_start = q_init
    for edge in list_of_edges:
        res,qi1,pid = generatePathOnEdge(edge,q_start,True)
        if not res: break
        paths_qinit.append(pid)
        q_start = qi1
    if not res: 
        print("failure")
        continue
        
    # put the ball in q_goal
    print ("put the ball in q_goal")
    q_start = q_goal
    for edge in list_of_edges:
        res,qi2,pid = generatePathOnEdge(edge,q_start,False)
        if not res: break
        paths_qgoal.append(pid)
        q_start = qi2
    if not res: 
        print("failure")
        continue
        
    #try to link the two paths
    print ("try to link the paths")
    res,pid,_ = ps.directPath(qi1,qi2,True)
    if not res: 
        print("failure")
        continue
    paths_qinit.append(pid)
    success = True
        
paths_qgoal.reverse()
paths = paths_qinit + paths_qgoal

print("solve done, let's display the path obtained")

v = vf.createViewer()
pp = PathPlayer(v)

for p in paths:
    pp(p)
