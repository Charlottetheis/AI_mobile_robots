from predicates import *
import copy
from actions import *
from objects import *
import copy


class State():
    def __init__(self, map_graph, g, predicates, agent_locations, object_locations, parent=None, action=None):
        
        self.map = map_graph #static predicates
        
        #for searching
        self.g = g 
        self.parent = parent
        self.action = action
        
        self.predicates = predicates #set
        self.agent_locations = agent_locations #dictionary
        self.object_locations = object_locations #dictionary
        
        
        
    def derive_state(self, actions):
        predicates = []
        
        for action in actions:
            predicates+=action.effects(self)
        
        nal, nol = self.update_locations(predicates)
        new_predicates = self.update_predicates(predicates)
        
        return State(self.map, self.g+1, new_predicates, nal, nol, parent=self, action=actions)
        
        
        
        
    def update_locations(self,predicates):
        new_agent_locations = self.agent_locations.copy()
        new_object_locations = self.object_locations.copy()
        
        for agent in new_agent_locations:
            if type(agent) == robot:
                new_agent_locations[agent] = new_agent_locations[agent].copy()
        
        for pred in predicates:
            if (type(pred) == AgentAt) and (type(pred.agent) == robot):
                new_agent_locations[pred.agent].append(pred.coordinate)
            elif (type(pred) == Not) and (type(pred.pred.agent) == robot):
                new_agent_locations[pred.pred.agent].remove(pred.pred.coordinate)
            elif (type(pred) == AgentAt):
                new_agent_locations[pred.agent] = pred.coordinate
            else:
                continue
                
        return new_agent_locations, new_object_locations
        
        
        
    def update_predicates(self,predicates):
        new_predicates = self.predicates.copy()
        for pred in predicates:
            if type(pred) == Not and pred.pred in new_predicates:
                new_predicates.remove(pred.pred)
                new_predicates.add(Free(pred.pred.coordinate))
            elif type(pred) == Not:
                new_predicates.add(Free(pred.pred.coordinate))
            elif pred.coordinate == (-1,-1):
                new_predicates.add(pred)
            elif Free(pred.coordinate) not in new_predicates: # When more than one concurrent conflict
                new_predicates.add(pred)
            else:
                new_predicates.add(pred)
                new_predicates.remove(Free(pred.coordinate))
        return new_predicates        
                
                
    def __eq__(self, other):
        return self.predicates == other.predicates

    def __hash__(self):
        return hash(frozenset(self.predicates))

    def __lt__(self, other):
        # This is used when adding States to a priority queue, to make agents not
        # contributing to the heuristic to prefer standing still
        if any([type(action)==NoOp for action in self.action]):
            return False
        else:
            return True              
      