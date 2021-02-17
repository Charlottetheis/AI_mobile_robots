from itertools import product, chain
from actions import *
from predicates import *
from heuristics import *
from state import *
from queue import PriorityQueue
import copy
from costmap import *

def replan_robot(true_state, robot, time, global_plan=None, G=None, look_ahead=4):
    
    #In case of AI planning robot is allowed to step where humans are initially located
    if G!=None:
        preds = []
        for pred in true_state.predicates:
            if (type(pred) == AgentAt) and (type(pred.agent)==human):
                preds.append(Not(pred))
        nal, nol = true_state.update_locations(preds)
        new_predicates = true_state.update_predicates(preds)
        state = State(true_state.map, true_state.g, true_state.t, new_predicates, nal, nol)
    else: state = true_state
        
    frontier = PriorityQueue()
    e_state = explored_state(state.predicates, [robot], state.action, state.t)
    #explored = {e_state}
    visited = {e_state: state.g}
    
    #if global_plan!=None:
    #    look_ahead = 1 #corresponding to 0 timesteps
    full_path = robot.plan_to_path(global_plan)
    goal_index = [full_path[1:].index(coors)+1 for coors in robot.goal]
    print(goal_index)
    for g in goal_index:
        if ((g-time) < 6) and ((g-time) > 0):
            look_ahead = g-time
    path = full_path[time+look_ahead:]
    heuristic = manhattan_to_path_robot(robot, path[:25])
    children = get_children(state, robot, G, time)
    
    for child in children:
        child_explored = explored_state(child.predicates, [robot], child.action, child.t)
        visited[child_explored] = child.g
        node = (heuristic(child)+child.g, child)
        frontier.put(node)

    while frontier.qsize() > 0:
        _,leaf = frontier.get()
        #print('time: ' + str(leaf.g))
        #print('agnet locations: ' + str([(agent,leaf.agent_locations[agent]) for agent in agents_in_conflict]))
        #print('paths ' + str([(agent, paths[i]) for i, agent in enumerate(agents_in_conflict)]))
        #leaf_explored = explored_state(leaf.predicates, [robot], leaf.action, leaf.t)
        #if leaf_explored in explored:
        #    continue
        #else: explored.add(leaf_explored)
        actions = backtrack(leaf, time)
        if set(leaf.agent_locations[robot]) in path:
            #counting the number of steps in the path that can be discarded
            back_at_path = path.index(set(leaf.agent_locations[robot]))+look_ahead
            old_path = full_path[:time+1] + full_path[time+back_at_path:]
            actions = backtrack(leaf, time)
            destinations = [action[0].destination for action in actions]
            #print(leaf.agent_locations[robot])
            #print(leaf.t)
            #print(actions)
            #print(destinations)
            #print(old_path)
            if (len(actions)>2) and all([goal in old_path+destinations for goal in robot.goal]):
            #print('agents: ' + str(agents_in_conflict))
            ##print('back at path: ' + str(back_at_path))
                #print('cost of final plan ' + str(leaf.g))
                return robot, actions, back_at_path 
         
        children = get_children(leaf, robot, G, time)
        #if all([coor in leaf.agent_locations[robot] for coor in [(2, 1), (2, 2), (1, 1), (1, 2)]]):
        #    print('robot on goal')
        #    print([(s.action, s.g) for s in children])
        for child in children:
            child_explored = explored_state(child.predicates, [robot], child.action, child.t)
            if (child.t < (time+25)) and ((child_explored not in visited) or (child.g < visited[child_explored])):
                visited[child_explored] = child.g
                node = (heuristic(child)+child.g, child)
                frontier.put(node)
                
    return robot, None, 0
    

def get_children(state, agent, G, time):
    children = []
    directions = [N(), S(), E(), W()]
    if G!=None:
        l_G = local_costmap(state, agent, state.t, G.copy(), true_time=time)
        action = NoOp(state, agent)
        if action.preconditions(state).issubset(state.predicates):
            child = state.derive_state([action],G=l_G)
            children.append(child)  
    else: l_G = G
    for direction in directions:
        action = MoveActionRobot(state, agent, direction)
        if action.preconditions(state).issubset(state.predicates):
            child = state.derive_state([action],G=l_G)
            children.append(child)          

    return children

    
def backtrack(s, time):
    actions = []
    while s.t > time:
        actions.append(s.action)
        s = s.parent
    actions.reverse()
    return actions
    
    
    
    

