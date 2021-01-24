import networkx as nx
import pygame
from sprites import *
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
from online_replanning_robot import *
from itertools import chain
from baseline_replan import *
from initialize import *
from viz_module import *
from costmap import *
from AI_replan import *
from AI_NN_replan import *
import pandas as pd



def simulate(m, h_n, experiments=[],viz=False):
    data = pd.DataFrame()#[[0,0,0,0,0,0,0,0,0,0,0,0,0,0]],columns=\
                        #['Baseline_Replan','Baseline_Rescue','Baseline_Close_1','Baseline_Close_2','Baseline_len','Baseline_h_len','Baseline_Robot_replan',\
#                         'AI_Replan','AI_Rescue','AI_Close_1','AI_Close_2','AI_len','AI_h_len','AI_Robot_replan'])
    
    G, sprite_walls = parse_map(m)
    
    G = global_costmap(G)
    
    humans, beds, predicates, object_locations = human_init(G, h_n)
    #print(object_locations)
    original_plans = []
    for h in humans:
        original_plans.append(h.plan.copy())
    
    for ex in experiments:
        if ex == 'Baseline':
            print(ex)
            r, start_pos = robot_init(G, costmap=None)
            type_replan = baseline_replan(r)
        if ex == 'AI':
            print(ex)
            r, start_pos = robot_init(G, costmap=G)
            type_replan = AI_replan(r, G)
        if ex == 'AINN':
            print(ex)
            r, start_pos = robot_init(G, costmap=G)
            type_replan = AI_NN_replan(r, G)
       
        ############### online part #################################
        not_finished = True
        t = 0

        #initialize state
        preds = predicates.copy()
        agent_locations = dict()
        for i, h in enumerate(humans):
            h.plan = original_plans[i].copy()
            h.replan_count = 0
            preds.add(AgentAt(h,(-1,-1)))
            agent_locations[h] = (-1,-1)
        preds.update([AgentAt(r, (x,y)) for (x,y) in r.get_full(start_pos)])
        preds.difference_update([Free((x,y)) for (x,y) in r.get_full(start_pos)])
        agent_locations[r] = r.get_full(start_pos)
        state = State(G,0,0,preds,agent_locations,object_locations)
        robot_stop = 0
        
        #statistics for evaluation
        replan_count = 0
        rescue_count = 0
        close_1_count = 0
        close_2_count = 0
        robot_replan_count = 0
        count_actions = 0
        count_wrongs = 0
        emergency_stop = 0
        
        while not_finished:
            # Finished robot plan
            if len(r.plan) <= t:
                not_finished = False
            else:
                #replanning for robot
                robot_replan_count_, count_actions_, count_wrongs_ = type_replan(state, t, robot_stop)
                
                #statistics
                robot_replan_count+=robot_replan_count_
                count_actions+=count_actions_
                count_wrongs+=count_wrongs_ 
                
                #All actions at time t
                actions_t = [h.plan[t-h.start_time] for h in humans if ((len(h.plan)-1+h.start_time) >= t) & (h.start_time <= t)]
                actions_t += [r.plan[t]] 
                
                # Checking consistency
                preconditions = [action.preconditions(state) for action in actions_t]
                destinations = [action.destination for action in actions_t[0:-1] if type(action)!=Leave]
                destinations += actions_t[-1].destination
                #print('preconditions: ' + str(preconditions))
                #print('does the precond hold: ' + str([precond in state.predicates for precond in chain.from_iterable(preconditions)]))
                #print('destinations: ' + str(destinations))
                #print('agent locations ' + str(state.agent_locations))

                #If consistent update state
                if all([precond in state.predicates for precond in chain.from_iterable(preconditions)]) \
                                    & (len(destinations) == len(set(destinations))):
                    print('Time: ' + str(t) + ' no conflicts')
                    state = state.derive_state(actions_t)
                    t += 1
                    robot_stop-=1
                    
                    #statistics
                    robot_location = state.agent_locations[r]
                    for h in humans:
                        h_coor = state.agent_locations[h]
                        dist = []
                        for coor in robot_location:
                            dist.append(euclidean_dist(h_coor, coor))
                        if min(dist) == 1:
                            close_1_count += 1
                        elif min(dist) < 2:
                            close_2_count += 1   
                                
                            
                            

                else:
                    # Replanning with two humans or one human+passive robot (stopping or continueing)
                    # The robot stps when a human is very close to it
                    print('replanning at time: ' + str(t))
                    agents = humans + [r]
                    agents_replanned, actions, back_at_path, robot_stop, res_c = replan(state,agents,t, G)
                    rescue_count += res_c #count number of resucue plans
                    print('agents replanned ' + str(agents_replanned))
                    #print('Actions replanned ' + str(actions))
                    robot_involved=any([type(a)==robot for a in agents_replanned])
                    for i, h in enumerate(agents_replanned):
                        if type(h)==robot:
                            replan_count += 1 #count number of replans
                        if (type(h)==robot) and (robot_stop==True):
                            print('robot stopped')
                            emergency_stop += 1
                            robot_stop=4
                            h.plan[t-h.start_time:t-h.start_time] = [NoOp(state, h)]*4
                        elif type(h)!=robot:
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
                            if robot_involved:
                                h.replan_count += 1 #counting towards using rescue planning



        #visualization module
        if viz:
            visualise(r, humans, beds, sprite_walls)                

        #Save statistics
        data.at[0,ex+'_Replan'] = replan_count
        data.at[0,ex+'_Rescue'] = rescue_count
        data.at[0,ex+'_Emergency_stop'] = emergency_stop
        data.at[0,ex+'_Close_1'] = close_1_count
        data.at[0,ex+'_Close_2'] = close_2_count
        data.at[0,ex+'_len'] = len(r.plan)
        h_len = 0
        for h in humans:
            h_len += len(h.plan)
        h_len/=len(humans)
        data.at[0,ex+'_h_len'] = h_len
        data.at[0,ex+'_Robot_replan'] = robot_replan_count
        data.at[0,ex+'_accuracy'] = (count_actions-count_wrongs)/count_actions
        
    return data
       
