from itertools import product, chain
from actions import *
from predicates import *
from heuristics import *
from state import *
from queue import PriorityQueue
import copy

def replan_robot(state, robot, time, global_plan=None):
    
    
    
    frontier = PriorityQueue()
    explored = {explored_state(state.predicates, [robot])}
    
    if global_plan==None:
        look_ahead = 4 # corresponding to 3 timesteps
    else: look_ahead = 1 #corresponding to 0 timesteps
    full_path = robot.plan_to_path(global_plan)
    path = full_path[time+look_ahead:]

    
    heuristic = manhattan_to_path_robot(robot, path)
    
    children = get_children(state, robot)
    
    for child in children:
        explored.add(child)
        node = (heuristic(child), child)
        frontier.put(node)

    while frontier.qsize() > 0:
        _,leaf = frontier.get()
        #print('time: ' + str(leaf.g))
        #print('agnet locations: ' + str([(agent,leaf.agent_locations[agent]) for agent in agents_in_conflict]))
        #print('paths ' + str([(agent, paths[i]) for i, agent in enumerate(agents_in_conflict)]))
        
        if set(leaf.agent_locations[robot]) in path:
            #counting the number of steps in the path that can be discarded
            back_at_path = path.index(set(leaf.agent_locations[robot]))+look_ahead
            old_path = full_path[:time] + full_path[time+look_ahead+back_at_path:]
            if all([goal in old_path for goal in robot.goal]):
            #print('agents: ' + str(agents_in_conflict))
            ##print('back at path: ' + str(back_at_path))
                return robot, backtrack(leaf, time), back_at_path 
         
        children = get_children(leaf, robot)
        for child in children:
            child_explored = explored_state(child.predicates, [robot])
            if (child_explored not in explored) and (child.g < (time+20)):
                explored.add(child_explored)
                node = (heuristic(child)+child.g, child)
                frontier.put(node)
                
    return robot, None, 0
    

def get_children(state, agent):
    children = []
    directions = [N(), S(), E(), W()]
    for direction in directions:
        action = MoveActionRobot(state, agent, direction)
        if action.preconditions(state).issubset(state.predicates):
            child = state.derive_state([action])
            children.append(child)          

    return children

    
def backtrack(s, time):
    actions = []
    while s.g > time:
        actions.append(s.action)
        s = s.parent
    actions.reverse()
    return actions
    
    
    
    

