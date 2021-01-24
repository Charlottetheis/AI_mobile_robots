import pandas as pd
from NN_helpers import *
from itertools import chain
from actions import *

def predict(state, agent, r, t, correct_action=None):
    if correct_action!=None:
        r.predictions[(agent, t-1)] = correct_action
        for time in range(0,15):
            p = prediction(agent, t+time)
            if (p!=None) and (type(p)==MoveAction):
                r.predictions[(agent, t+time)] = p.destination
                r.dir_predictions[(agent, t+time)] = dir_to_num(p.direction)
        
        
    else:
        for time in range(15):
            if (agent, t+time) not in r.predictions:
                p = prediction(agent, t+time)
                if (p!=None) and (type(p)==MoveAction):
                    r.predictions[(agent, t+time)] = p.destination
                    r.dir_predictions[(agent, t+time)] = dir_to_num(p.direction)

        
             
def prediction(agent, t):
    if len(agent.plan)+agent.start_time > t:
        return agent.plan[t-agent.start_time]
    else: return None
    
    
#Prediction using the NN
def predict_NN(state, agent, r, t, model, correct_action=None):
    if correct_action!=None:
        r.predictions[(agent, t-1)] = correct_action.destination
        r.dir_predictions[(agent, t-1)] = dir_to_num(correct_action.direction)
    agent_dic = dict()
    for pred in r.predictions.keys():
        if pred[1] in agent_dic:
            agent_dic[pred[1]].append(r.predictions[pred])
        else:
            agent_dic[pred[1]]=[r.predictions[pred]]
    df = pd.DataFrame(columns=['time','query_state','sample1','sample2','sample3','sample4','sample5','direction'])
    df.time = list(range(15))
    #print(r.predictions)
    #exits
    exit = [a[0] for a in state.map.nodes('type') if a[1]=='e']
    #if prediction was wrong; predict all over
    if correct_action!=None:
        for time in range(15):
            if r.predictions[(agent, time+t-1)] in exit: 
                break
            else:
                prediction_NN(df, state, t, time, agent_dic, agent, r, model)
            
    #Only predict the missing time step; usually just the tenth, or all 10 for first time prediction
    else:
        for time in range(15):
            if r.predictions[(agent, time+t-1)] in exit:
                break
            elif (agent, t+time) not in r.predictions:
                prediction_NN(df, state, t, time, agent_dic, agent, r, model)
                
    
def prediction_NN(df, state, t, time, agent_dic, agent, r, model):
    obj_dic = list(chain.from_iterable(state.object_locations.values()))
    df.at[time,'query_state'] = new_state(state.map, r.predictions[(agent, t+time-1)], obj_dic, agent_dic, t+time)
    df.at[time,'sample1'] = new_state(state.map, r.predictions[(agent, t+time-2)], obj_dic, agent_dic, t+time-1)
    df.at[time,'sample1'][-1][r.dir_predictions[(agent, t+time-2)]] = 1                                      
    df.at[time,'sample2'] = new_state(state.map, r.predictions[(agent, t+time-3)], obj_dic, agent_dic, t+time-2)
    df.at[time,'sample2'][-1][r.dir_predictions[(agent, t+time-3)]] = 1                                      
    df.at[time,'sample3'] = new_state(state.map, r.predictions[(agent, t+time-4)], obj_dic, agent_dic, t+time-3)
    df.at[time,'sample3'][-1][r.dir_predictions[(agent, t+time-4)]] = 1
    df.at[time,'sample4'] = new_state(state.map, r.predictions[(agent, t+time-5)], obj_dic, agent_dic, t+time-4)
    df.at[time,'sample4'][-1][r.dir_predictions[(agent, t+time-5)]] = 1
    df.at[time,'sample5'] = new_state(state.map, r.predictions[(agent, t+time-6)], obj_dic, agent_dic, t+time-5)
    df.at[time,'sample5'][-1][r.dir_predictions[(agent, t+time-6)]] = 1
    df.at[time,'direction'] = -1
    sample = transform(df, time)
    output = model(sample)
    
    dir_prediction = torch.sort(output['out'], 1, descending=True)[1]
    for direction in dir_prediction[0]:
        loc = pred_to_action(direction, r.predictions[(agent, t+time-1)])
        if (loc in state.map.nodes()) and (loc not in obj_dic):
            break
    
    r.dir_predictions[(agent, t+time)] = direction
    #print(loc)
    #print(output)
    r.predictions[(agent, t+time)] = loc
    if time+t in agent_dic:
        agent_dic[time+t].append(loc)
    else: agent_dic[time+t]=loc 
    