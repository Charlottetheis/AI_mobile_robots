from itertools import product, chain
from actions import *
from predicates import *
from heuristics import *
from state import *
from queue import PriorityQueue
import copy
from online_replanning_robot import *

def baseline_replan(r):
    def replanning(state, t, robot_stop):
        replan_count = 0
        if robot_stop<=0:
            # replanning in case obstacle moved
            if t in r.obstacle:
                obstacle_coors = [Free(x) for x in r.obstacle[t]]
                if all([coor in state.predicates for coor in obstacle_coors]):
                    #print('obstacle has moved at time ' + str(t))
                    rx, new_actions, back_at_path = replan_robot(state, r, t, \
                                                                global_plan=r.previous_plan)
                    #print('New actions robot ' +str(new_actions))
                    #print('Back at path robot ' +str(back_at_path))
                    if (new_actions!=None) and ((back_at_path+t)<r.back_at_global_path[t]):
                        new_actions = [action[0] for action in new_actions]
                        # Update the current plan
                        del r.plan[t-r.start_time:]
                        r.plan += new_actions
                        r.plan += r.previous_plan[t+back_at_path:]
                        r.previous_plan = r.plan.copy()



                        #update history
                        for i in range(t,r.detour[t][1]):
                            del r.obstacle[i]
                            del r.detour[i]
                            del r.back_at_global_path[i]
                        r.replanning[t] = r.plan_to_path()


            else:        
                # Replanning for robots in case of objects in path
                human_loc = [pred.coordinate for pred in state.predicates if (type(pred)==AgentAt) and (pred.agent!=r)]
                object_loc = [pred.coordinate for pred in state.predicates if (type(pred)==ObjectAt)]
                all_loc = human_loc + [coor for coor in chain.from_iterable(object_loc)]
                r_path = r.plan_to_path()
                r_path = r_path[t+1:t+4]
                if any([coor in chain.from_iterable(r_path) for coor in all_loc]):

                    #print('Something in robot path at ' + str(t))
                    replan_count = 1
                    rx, new_actions, back_at_path = replan_robot(state, r, t)
                    if new_actions == None:
                        new_actions = [NoOp(state, r)]*4
                    else:
                        new_actions = [action[0] for action in new_actions]
                    #print('New actions robot ' +str(new_actions))
                    #print('Back at path robot ' +str(back_at_path))

                    # Update the current plan
                    r.previous_plan = r.plan.copy()
                    del r.plan[t-r.start_time:back_at_path+t-r.start_time]
                    r.plan[t-r.start_time:t-r.start_time] = new_actions

                    #update history
                    obs_coor = [coor for coor in all_loc if (coor in chain.from_iterable(r_path))]
                    for i in range(0,len(new_actions)):
                        r.obstacle[t+i] = obs_coor 
                        r.detour[t+i] = [t,t+len(new_actions)] #[start,finish]
                        r.back_at_global_path[t+i] = t+back_at_path
                    r.replanning[t] = r.plan_to_path()
        return replan_count, 1 , 1
    return replanning