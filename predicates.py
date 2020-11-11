#predicates to store information on dynamic state variables
#AgentAt(id,coor)
#Not(predicate)



class predicate():
    def __eq__(self, other):
        return hash(self) == hash(other)
    
    
class AgentAt(predicate):
    def __init__(self, agent_id, coordinate):
        self.agent = agent_id
        self.coordinate = coordinate
     
    def __repr__(self):
        return 'AgentAt({}, {})'.format(self.agent, self.coordinate)

    def __hash__(self):
        return hash(('agentat',self.agent, self.coordinate))
    
class ObjectAt(predicate):
    def __init__(self, object_id, coordinate):
        self.id = object_id
        self.coordinate = coordinate
     
    def __repr__(self):
        return 'ObjectAt({}, {})'.format(self.id, self.coordinate)

    def __hash__(self):
        return hash(('objecttat',self.id, *self.coordinate))
    
    
class Not(predicate):
    def __init__(self, pred):
        self.pred = pred
     
    def __repr__(self):
        return 'Not({})'.format(self.pred)

    def __hash__(self):
        return hash(('not',self.pred))

    
    
class Free(predicate):
    def __init__(self,coordinate):
        self.coordinate = coordinate
        
    def is_valid(self, state):
        return (self.coordinate in state.agent_locations.values) or (self.coordinate in state.object_locations.values)
        
    def __repr__(self):
        return 'Free({})'.format(self.coordinate)

    def __hash__(self):
        return hash(('free',self.coordinate))