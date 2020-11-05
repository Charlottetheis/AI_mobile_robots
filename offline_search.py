
#single agent search to use for finding global plans

from predicates import *
from heuristics import *
from state import *
from queue import PriorityQueue
from actions import *



def offline_search(state, agent, start, goal, heuristic):
    
    frontier = PriorityQueue()
    explored = {state}
    
    children = get_children(state, agent)
    
    for child in children:
        explored.add(child)
        node = (heuristic(child), child)
        frontier.put(node)
    
    
    while frontier.qsize() > 0:
        _,leaf = frontier.get()
        if leaf.agent_locations[agent] == goal:
            return backtrack(leaf)
        
        
        children = get_children(leaf, agent)
        for child in children:
            if child not in explored:
                explored.add(child)
                node = (heuristic(child)+child.g, child)
                frontier.put(node)
                
    return 'Error'
    
def get_children(state, agent):
    children = []
    directions = [N(), S(), E(), W()]  
    for direction in directions:
        action = MoveAction(state, agent, direction)
        if action.preconditions().issubset(state.predicates):
            child = state.derive_state([action])
            children.append(child)          

    return children


def backtrack(state):
    actions = []
    while state.parent is not None:
        actions+=state.action
        state = state.parent
    actions.reverse()
    return actions
    
    
    
    