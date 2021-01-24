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
        replan_activated_human = False
        replan_activated_obj = False
        new_predictions = False
        replan_count = 0
        count_wrongs = 0
        count_actions = 0
        new_predictions = []
        #Loading NN
        model = Net()
        model.load_state_dict(torch.load('network2'))
        model.eval()
        
        if state.action!=None:
            for action in state.action:
                delete_list = []
                if type(action)==Leave:
                    for agent, time in r.predictions:
                        if agent == action.agent:
                            delete_list.append((agent, time))
                    for item in delete_list:
                        del r.predictions[item]
                elif (type(action.agent)==human) and (type(action)!=Enter):
                    #save the correct step
                    if ((action.agent, t-1) not in r.predictions) or ((action.agent, t-6) not in r.predictions):
                        r.predictions[(action.agent, t-1)] = action.destination
                        r.dir_predictions[(action.agent, t-1)] = dir_to_num(action.direction)
                        if all([(action.agent, t-x) in r.predictions for x in [2,3,4,5,6]]):
                            print('predictions for new agent')
                            predict_NN(state, action.agent, r, t, model)# Predict ten time steps ahead
                            
                    else: #check if prediction is correct 
                        count_actions+=1
                        if action.destination != r.predictions[(action.agent, t-1)]:
                            print('wrong prediction')
                            count_wrongs += 1
                            new_predictions.append(action.agent)
                            predict_NN(state, action.agent, r, t, model, correct_action=action) #repredict


                        predict_NN(state, action.agent, r, t, model) # Predict ten time steps ahead
            #print(r.predictions)
            if robot_stop <= 0:
                delete_list = []
                agent = None
                for agent, time in r.predictions:
                    if time >= t:
                        if (time in r.obstacle) and (r.obstacle[time][0] in new_predictions):
                            print('Replan caused by wrong prediction at time ' + str(time))
                            replan_activated_human=True
                            break
                        elif (time in [t, t+1, t+2, t+3]) and (len(r.plan)>time):
                            position = r.plan[time].destination
                            x_coor = min({coor[0] for coor in position})
                            y_coor = min({coor[1] for coor in position})
                            if (r.predictions[(agent, time)][0] in \
                                list(range(x_coor-1,x_coor+3))) and (r.predictions[(agent, time)][1] in \
                                                                     list(range(y_coor-1,y_coor+3))):

                                replan_activated_human=True

                                print('Human too close to path at time ' + str(time))
                                break
                    elif time < t-6:
                        delete_list.append((agent, time))

                for item in delete_list:
                    del r.predictions[item]

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