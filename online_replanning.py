from itertools import product, chain
from actions import *
from predicates import *
from heuristics import *
from state import *
from queue import PriorityQueue
import copy

def replan(state, agents, time):
    
    
    
    frontier = PriorityQueue()
    agents_in_conflict, agents_not_in_conflict = conflict(state,agents,time)
    explored = {explored_state(state.predicates, agents_in_conflict)}
    
    paths = []
    agents_in_conflict_no_robot = []
    for agent in agents_in_conflict:
        if type(agent)!=robot:
            path = agent.plan_to_path()
            paths.append(path[time-agent.start_time+1:])
            agents_in_conflict_no_robot.append(agent)
        else:
            paths.append([])
    
    heuristic = manhattan_to_path(agents_in_conflict_no_robot, paths)
    
    children = get_children(state, agents_in_conflict, agents_not_in_conflict)
    
    for child in children:
        explored.add(child)
        node = (heuristic(child), child)
        frontier.put(node)

    while frontier.qsize() > 0:
        _,leaf = frontier.get()
        #print('time: ' + str(leaf.g))
        #print('agnet locations: ' + str([(agent,leaf.agent_locations[agent]) for agent in agents_in_conflict]))
        #print('paths ' + str([(agent, paths[i]) for i, agent in enumerate(agents_in_conflict)]))
        
        if all([leaf.agent_locations[agent] in paths[i] for i, agent in enumerate(agents_in_conflict) if type(agent)!=robot]):
            #counting the number of steps in the path that can be discarded
            back_at_path = [paths[i].index(leaf.agent_locations[agent])+1 if type(agent)!=robot else 0 \
                            for i, agent in enumerate(agents_in_conflict)]
            #print('agents: ' + str(agents_in_conflict))
            ##print('back at path: ' + str(back_at_path))
            return agents_in_conflict, backtrack(leaf, time), back_at_path 
         
        children = get_children(leaf, agents_in_conflict, agents_not_in_conflict)
        for child in children:
            child_explored = explored_state(child.predicates, agents_in_conflict)
            if child_explored not in explored:
                explored.add(child_explored)
                node = (heuristic(child)+child.g, child)
                frontier.put(node)
                
    return agents_in_conflict, None, None
    
    
    
def conflict(state, agents, time):
    agents_in_conflict = []
    agents_not_in_conflict = []
    for h1 in range(len(agents)):
        for h2 in range(h1+1,len(agents)):
            res1 = agents[h1].get_resources(state,time)
            res2 = agents[h2].get_resources(state,time)
            if any([coor in res1 for coor in res2]):
                agents_in_conflict.append(agents[h1])
                agents_in_conflict.append(agents[h2])
                
                for h in agents:
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
    #else:
        #actions = gen_actions(agents_in_conflict[0], state)
        #combinations = [[action] for action in actions]
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
    if type(agent) == robot:
        return [NoOp(state,agent)]
    else:
        actions = []
        directions = [N(), S(), E(), W()]
        for direction in directions:
            actions.append(MoveAction(state, agent, direction))
        actions += [NoOp(state, agent), Leave(agent), Enter(agent)]
        return actions


def valid(combo, actions_no_conflict):
    #destinations for agents not in the conflict
    destinations_no_conflict = [action.destination for action in actions_no_conflict if \
                                (type(action)!=Leave) and (type(action.agent)!=robot)] 
    destination_no_conflict_robot = [action.destination for action in actions_no_conflict if type(action.agent)==robot]
    destinations_no_conflict += [coor for coor in chain.from_iterable(destination_no_conflict_robot)]
    
    #destination for the two agents in conflict
    destinations = [action.destination for action in combo if (type(action)!=Leave) and (type(action.agent)!=robot)]
    destination_robot = [action.destination for action in combo if type(action.agent)==robot]
    destinations_all = destinations + [coor for coor in chain.from_iterable(destination_robot)]
    if (len(destinations_all) != len(set(destinations_all))) or (any([dest in destinations_no_conflict for dest in destinations])):
        return False
    else:
        return True
    
    
def backtrack(s, time):
    actions = []
    while s.g > time:
        actions.append(s.action)
        s = s.parent
    actions.reverse()
    return actions
    
    
    
    

