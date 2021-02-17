from predicates import *
import copy
from actions import *
from objects import *
import copy
from itertools import chain


class State():
    def __init__(self, map_graph, g, t, predicates, agent_locations, object_locations, parent=None, action=None):
        
        self.map = map_graph #static predicates
        
        #for searching
        self.g = g
        self.t = t
        self.parent = parent
        self.action = action
        
        self.predicates = predicates #set
        self.agent_locations = agent_locations #dictionary
        self.object_locations = object_locations #dictionary
        
        
        
    def derive_state(self, actions, G=None):
        predicates = []
        
        for action in actions:
            predicates+=action.effects(self)
        
        nal, nol = self.update_locations(predicates)
        new_predicates = self.update_predicates(predicates)
        
        c = 1
        if G!= None:
            for action in actions:
                loc = nal[action.agent]
                if type(action.agent) == human:
                    
                    c += G.nodes[loc]['cost']
                    
                else:
                    if set(loc) in action.agent.goal:
                        c+=0 #no extra cost to step into goal field
                    else:       
                        if type(actions[0]) != NoOp:
                            c += 0.5
                            for l in loc:
                                c += G.nodes[l]['cost']
                        else:
                            for l in loc:
                                c += G.nodes[l]['cost']
        
        return State(self.map, self.g+c, self.t+1, new_predicates, nal, nol, parent=self, action=actions)
        
        
        
        
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
            if (type(pred) == Not) and (pred.pred.coordinate in chain.from_iterable(self.object_locations.values())):
                print('Not removing object')
                print(pred)
            elif (type(pred) == Not) and (pred.pred in new_predicates):
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

        
class explored_state():
    def __init__(self, predicates, agents, actions=None, t=0):
        sp = {pred for pred in predicates if (type(pred) == AgentAt) and (pred.agent in agents)}
        sp.add(t)
                                 
        self.selected_predicates = sp
        self.action = actions
        
    def __eq__(self, other):
        return self.selected_predicates == other.selected_predicates

    def __hash__(self):
        return hash(frozenset(self.selected_predicates))

    def __lt__(self, other):
        if type(self.action) == NoOp:
            return False
        else:
            return True
        # This is used when adding States to a priority queue, to make agents not
        # contributing to the heuristic to prefer standing still   
        
        
        
        
        