
from graph_parser import *
import random
import sys
from objects import *
import numpy as np
from predicates import *
from state import *
from offline_search import *
from actions import *
from online_replanning import *
from itertools import chain
from baseline_replan import *
from initialize import *
from costmap import *
import pandas as pd



def generate(m, h_n):
    G, sprite_walls = parse_map(m)
    
    G = global_costmap(G)
    
    humans_original, beds, predicates, object_locations = human_init(G, h_n)
       
    ############### online part #################################
    not_finished = True
    t = 0

    #initialize state
    preds = predicates.copy()
    agent_locations = dict()
    humans = humans_original.copy()
    for h in humans:
        preds.add(AgentAt(h,(-1,-1)))
        agent_locations[h] = (-1,-1)
    state = State(G,0,0,preds,agent_locations,object_locations)
    l = [(x,y) for x in list(range(-5,6)) for y in list(range(-5,6)) if (x,y)!=(0,0)]
    dir_dic = {'N': 0, 'S': 1, 'E': 2, 'W': 3, 'None': 4}
    flat_obj_loc=list(chain.from_iterable(object_locations.values()))
    data = pd.DataFrame(columns=['agent','direction','time','true_time','agent_loc', 'query_state']) 
    while not_finished:
        # Finished robot plan
        if (t>0) and (len(actions_t) == 0):
            not_finished = False
        else:

            #All actions at time t
            actions_t = [h.plan[t-h.start_time] for h in humans if ((len(h.plan)-1+h.start_time) >= t) & (h.start_time <= t)]

            # Checking consistency
            preconditions = [action.preconditions(state) for action in actions_t]
            destinations = [action.destination for action in actions_t if type(action)!=Leave]

            #If consistent update state
            if all([precond in state.predicates for precond in chain.from_iterable(preconditions)]) \
                                & (len(destinations) == len(set(destinations))):
                
                
                agent_loc = state.agent_locations.values()
                d = []
                for act in actions_t:
                    if (type(act) == MoveAction) or (type(act) == NoOp):
                        w = []
                        o = []
                        a = []
                        pos = state.agent_locations[act.agent]
                        for loc in l:
                            coor = (pos[0]+loc[0],pos[1]+loc[1])
                            if coor not in G.nodes:
                                w.append(loc)
                            elif coor in flat_obj_loc:
                                o.append(loc)
                            elif coor in agent_loc:
                                a.append(loc)
                        direction_vec = [0,0,0,0,0]
                        direction_vec[dir_dic[str(act.direction)]] = 1
                        
                        d.append([act.agent.human_id, direction_vec, state.t-act.agent.start_time, state.t, pos, str([w, o, a, direction_vec])])

                df = pd.DataFrame(d, columns=['agent','direction','time','true_time','agent_loc','query_state'])
                data = data.append(df,ignore_index=True)

                state = state.derive_state(actions_t)
                t += 1
            else:
                # Replanning with two humans or one human+passive robot (stopping or continueing)
                # The robot stops when a human is very close to it
                agents = humans
                agents_replanned, actions, back_at_path, robot_stop, _ = replan(state,agents,t, G)
                for i, h in enumerate(agents_replanned):
                    if (actions != None) and (type(actions[0]) == list):
                        new_actions = [action for action in chain.from_iterable(actions) if action.agent==h]

                        del h.plan[t-h.start_time:back_at_path[i]+t-h.start_time]

                    elif actions != None: #in rare case of offline replanning

                        new_actions = actions
                        del h.plan[t-h.start_time:-1]
                    else: # In case no solution was found
                        new_actions = [NoOp(state, h)]
                        #print('new actions: ' + str(new_actions))
                        #print('new actions preconditions: ' + str([action.preconditions(action.state) for action in new_actions]))
                        #print('back at path: ' + str(back_at_path[i]))
                    h.plan[t-h.start_time:t-h.start_time] = new_actions
                    h.replan_count += 1


    return data, G, flat_obj_loc