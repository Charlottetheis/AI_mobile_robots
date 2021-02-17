from itertools import product, chain
from actions import *
from predicates import *
from heuristics import *
from state import *
from queue import PriorityQueue
import copy
from online_replanning_robot import *
from prediction import *
from NN_helpers import *

def AI_NN_replan(r, G):
    def replanning(state, t, robot_stop):
        print(r.predictions)
        replan_activated_human = False
        replan_activated_obj = False
        new_predictions = False
        replan_count = 0
        count_wrongs = 0
        count_actions = 0
        new_predictions = []
        exit = [a[0] for a in G.nodes('type') if a[1]=='e']
        #Loading NN
        model = Net()
        model.load_state_dict(torch.load('network2'))
        model.eval()
        
        if state.action!=None:
            for action in state.action:
                delete_list = []
                if (type(action)==Leave) or ((action.destination in exit) and (t-action.agent.start_time > 10)):
                    print(action.agent)
                    print('is leaving at time ' + str(t))
                    for agent, time in r.predictions:
                        if agent == action.agent:
                            delete_list.append((agent, time))
                    for item in delete_list:
                        del r.predictions[item]
                        del r.dir_predictions[item]
                        if item in r.probs_predictions:
                            del r.probs_predictions[item]
                        
                elif (type(action.agent)==human) and (type(action)!=Enter) and (action.destination!=(-1,-1)):
                    #save the correct step
                    if (t-action.agent.start_time < 8) or ((action.agent, t-1) not in r.predictions):
                        print(action.agent)
                        print(action)
                        print(action.agent.start_time)
                        print([(action.agent, t-x) in r.predictions for x in [2,3,4,5,6]])
                        r.predictions[(action.agent, t-1)] = action.destination
                        direction = dir_to_num(action.direction)
                        r.dir_predictions[(action.agent, t-1)] = direction
                        
                        if all([(action.agent, t-x) in r.predictions for x in [2,3,4,5,6]]):
                            print('predictions for new agent')
                            predict_NN(state, action.agent, r, t, model, correct_action=action)# Predict 15 time steps ahead
                        else:
                            probs = [0,0,0,0,0]
                            probs[direction] = 1
                            for x in range(1,4):
                                if (action.agent, t+x-2) in r.predictions:
                                    loc = pred_to_action(direction,r.predictions[(action.agent, t+x-2)])
                                    if loc in G:
                                        r.predictions[(action.agent, t+x-1)] = loc
                                        r.dir_predictions[(action.agent, t+x-1)] = direction
                                        r.probs_predictions[(action.agent, t+x-1)] = probs
                        
                            
                    else: #check if prediction is correct 
                        count_actions+=1
                        if action.destination != r.predictions[(action.agent, t-1)]:
                            print('wrong prediction')
                            count_wrongs += 1
                            new_predictions.append(action.agent)
                            predict_NN(state, action.agent, r, t, model, correct_action=action) #repredict


                        else:
                            predict_NN(state, action.agent, r, t, model) # Predict ten time steps ahead
            #print(r.predictions)
            if robot_stop <= 0:
                delete_list = []
                agent = None
                for agent, time in r.predictions:
                    if time >= t:
                        if (time in r.obstacle) and (r.obstacle[time][0] in new_predictions):
                            agent = r.obstacle[time][0]
                            print('Replan caused by wrong prediction at time ' + str(time))
                            replan_activated_human=True
                        elif (time in [t, t+1, t+2, t+3]) and (len(r.plan)>time):
                            position = r.plan[time].destination
                            x_coor = min({coor[0] for coor in position})
                            y_coor = min({coor[1] for coor in position})
                            if time==t:
                                for i, prob in enumerate(r.probs_predictions[(agent, time)]):
                                    loc = pred_to_action(i,r.predictions[(agent, time-1)])
                                    if (prob > 0.1) and (loc[0] in list(range(x_coor-1,x_coor+3))) and (loc[1] in \
                                                                                                      list(range(y_coor-1,y_coor+3))):
                                        replan_activated_human=True
                                        print('Human probably too close to path at time ' + str(time))
                            elif (r.predictions[(agent, time)][0] in \
                                list(range(x_coor-1,x_coor+3))) and (r.predictions[(agent, time)][1] in \
                                                                     list(range(y_coor-1,y_coor+3))):
                                replan_activated_human=True
                                print('Human too close to path at time ' + str(time))
                    elif time < t-6:
                        delete_list.append((agent, time))
                    
                    if replan_activated_human:
                        break

                for item in delete_list:
                    del r.predictions[item]
                    del r.dir_predictions[item]
                    if item in r.probs_predictions:
                        del r.probs_predictions[item]

                # Replanning for robots in case of objects in path
                object_loc = [pred.coordinate for pred in state.predicates if (type(pred)==ObjectAt)]
                r_path = r.plan_to_path()
                r_path = r_path[t+1:t+4]
                if any([coor in chain.from_iterable(r_path) for coor in chain.from_iterable(object_loc)]):
                    replan_activated_obj = True
                    print('object in path at time ' + str(t))
                    print([coor for coor in chain.from_iterable(object_loc) if coor in chain.from_iterable(r_path)])
                    print(state.agent_locations[r])
                    print(r.plan[t])

                if (replan_activated_human) or (replan_activated_obj):
                    replan_count = 1

                    rx, new_actions, back_at_path = replan_robot(state, r, t, global_plan=None, G=G, look_ahead=6)
                    #r.add_actions+=(back_at_path-len(new_actions))
                    #print(r.add_actions)
                    if new_actions == None:
                        new_actions = [NoOp(state, r)]*4
                    else:
                        new_actions = [action[0] for action in new_actions]
                    print('New actions robot ' +str(new_actions))
                    print('Back at path robot ' +str(back_at_path))

                    # Update the current plan
                    r.previous_plan = r.plan.copy()
                    del r.plan[t-r.start_time:back_at_path+t-r.start_time]
                    r.plan[t-r.start_time:t-r.start_time] = new_actions 
                    
                    if replan_activated_human:
                        for i in range(len(new_actions)):
                            r.obstacle[t+i] = [agent, r.predictions[(agent, t)]]

        return replan_count, count_actions, count_wrongs
    return replanning