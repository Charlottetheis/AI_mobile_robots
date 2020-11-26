from itertools import product, chain
from actions import *
from predicates import *
from heuristics import *
from state import *
from queue import PriorityQueue
import copy

def replan(state, humans, time):
    
    
    
    frontier = PriorityQueue()
    explored = {state}
    

    
    agents_in_conflict, agents_not_in_conflict = conflict(state,humans,time)

    
    paths = []
    for agent in agents_in_conflict:
        path = agent.plan_to_path()
        paths.append(path[time-agent.start_time+1:])
    
    heuristic = manhattan_to_path(agents_in_conflict, paths)
    
    children = get_children(state, agents_in_conflict, agents_not_in_conflict)
    
    for child in children:
        explored.add(child)
        node = (heuristic(child), child)
        frontier.put(node)
    
    while frontier.qsize() > 0:
        _,leaf = frontier.get()
        #print('time: ' + str(leaf.g))
        #print('agnet locations: ' + str([(agent,leaf.agent_locations[agent]) for agent in agents_in_conflict]))
        
        if all([leaf.agent_locations[agent] in paths[i] for i, agent in enumerate(agents_in_conflict)]):
            #counting the number of steps in the path that can be discarded
            back_at_path = [paths[i].index(leaf.agent_locations[agent])+1 for i, agent in enumerate(agents_in_conflict)]
            return agents_in_conflict, backtrack(leaf, time), back_at_path 
         
        children = get_children(leaf, agents_in_conflict, agents_not_in_conflict)
        for child in children:
            if child not in explored:
                explored.add(child)
                node = (heuristic(child)+child.g, child)
                frontier.put(node)
                
    return 'Error'
    
    
    
def conflict(state, humans, time):
    agents_in_conflict = []
    agents_not_in_conflict = []
    for h1 in range(len(humans)):
        for h2 in range(h1+1,len(humans)):
            res1 = humans[h1].get_resources(state,time)
            res2 = humans[h2].get_resources(state,time)
            if any([coor in res1 for coor in res2]):
                human1 = humans[h1]
                human2 = humans[h2]
                agents_in_conflict.append(humans[h1])
                agents_in_conflict.append(humans[h2])
                
                for h in humans:
                    if h not in agents_in_conflict:
                        agents_not_in_conflict.append(h)

                return agents_in_conflict, agents_not_in_conflict           
    return agents_in_conflict, agents_not_in_conflict 
    
    
def get_children(state, agents_in_conflict, agents_not_in_conflict):
    children = []
    actions = []
    for agent in agents_in_conflict:
        actions.append(gen_actions(agent, state))
    combinations = product(*actions)
    for combo in combinations:
        precond = {element for element in chain.from_iterable([action.preconditions(state) for action in combo])}
        actions_no_conflict = [agent.plan[state.g-agent.start_time] for agent in agents_not_in_conflict \
                                if ((len(agent.plan)-1+agent.start_time) >= state.g) and (agent.start_time <= state.g)] 
        
        if valid(combo, actions_no_conflict) & precond.issubset(state.predicates):
            all_actions = list(combo) + actions_no_conflict
            child = state.derive_state(all_actions)
            #print('time for child: ' + str(child.g) + ' for combo: ' + str(combo))
            children.append(child)          
    return children

def gen_actions(agent, state):
    actions = []
    directions = [N(), S(), E(), W()]
    for direction in directions:
        actions.append(MoveAction(state, agent, direction))
    actions += [NoOp(state, agent), Leave(agent), Enter(agent)]
    return actions


def valid(combo, actions_no_conflict):
    destinations_no_conflict = [action.destination for action in actions_no_conflict if type(action)!=Leave] 
    destinations = [action.destination for action in combo if type(action)!=Leave]
    #all_destinations = destinations_no_conflict+destinations
    if (len(destinations) != len(set(destinations))) or (any([dest in destinations_no_conflict for dest in destinations])):
        return False
    else:
        return True
    
    
def backtrack(state, time):
    actions = []
    while state.g > time:
        actions.append(state.action)
        state = state.parent
    actions.reverse()
    return actions
    
    
    
    

