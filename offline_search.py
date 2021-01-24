
#single agent search to use for finding global plans

from predicates import *
from heuristics import *
from state import *
from queue import PriorityQueue
from actions import *
from objects import *
from costmap import *



def offline_search(state, agent, goal, heuristic, G, time=0):
    frontier = PriorityQueue()
    explored = {state}
    visited = {state: 0}
    children = get_children(state, agent, G)
    
    for child in children:
        h = heuristic(child)
        visited[child] = child.g
        node = (h + child.g, child)
        frontier.put(node)

    
    
    while frontier.qsize() > 0:
        _,leaf = frontier.get()
        
        if leaf in explored:
            continue
        else:
            explored.add(leaf)
            
        if leaf.agent_locations[agent] == goal:
            return backtrack(leaf, time)
        elif len(goal)==4 and all([(x,y) in goal for (x,y) in leaf.agent_locations[agent]]):
            return backtrack(leaf, time)
        
        children = get_children(leaf, agent, G)
        for child in children:
            if (child not in visited) or (child.g < visited[child]):
                visited[child] = child.g
                node = (heuristic(child)+child.g, child)
                frontier.put(node)
    return None
    
def get_children(state, agent, G):
    children = []
    directions = [N(), S(), E(), W()]
    if G==None:
        l_G = None
    elif type(agent) == robot:
        l_G = local_costmap(state, agent, state.t, G.copy())
    else:
        l_G = local_costmap_human(state, agent, state.t, G.copy())
    for direction in directions:
        if type(agent) == human:
            action = MoveAction(state, agent, direction)
        else:
            action = MoveActionRobot(state, agent, direction)
        if action.preconditions(state).issubset(state.predicates):
            child = state.derive_state([action], l_G)
            children.append(child)          

    return children


def backtrack(state, time):
    actions = []
    while state.t > time:
        actions+=state.action
        state = state.parent
    actions.reverse()
    return actions
    
    
    
    