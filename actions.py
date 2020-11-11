from predicates import *

#DIRECTIONS
class Direction():

    def move(self, coordinate):
        return coordinate

class N(Direction):

    def move(self, coordinate):
        (x,y) = coordinate
        return (x,y-1)

    def __repr__(self):
        return 'N'

class W(Direction):

    def move(self, coordinate):
        (x,y) = coordinate
        return (x-1,y)

    def __repr__(self):
        return 'W'

class S(Direction):

    def move(self, coordinate):
        (x,y) = coordinate
        return (x,y+1)

    def __repr__(self):
        return 'S'

class E(Direction):

    def move(self, coordinate):
        (x,y) = coordinate
        return (x+1,y)

    def __repr__(self):
        return 'E'


# ACTIONS
class Action():

    def is_valid(self, state):
        return all([precond.is_valid(state) for precond in self.preconditions()])

class NoOp():

    def __init__(self, state, agent):
        self.agent = agent
        self.destination = state.agent_locations[self.agent]
        
    def preconditions(self):
        return {}

    def effects(self,state):
        return []

    def is_valid(self,state):
        return True

    def is_conflict(self,other):
        return False

    def __lt__(self,other):
        return True

    def __repr__(self):
        return "NoOp"

class MoveAction(Action):

    def __init__(self, state, agent, direction):
        self.agent = agent
        self.direction = direction
        self.state = state
        self.destination = direction.move(self.state.agent_locations[self.agent])
        self.box_destination = None

    def preconditions(self, state):
        destination = self.direction.move(state.agent_locations[self.agent])
        return {Free(destination)}

    def effects(self,state):
        destination = self.direction.move(state.agent_locations[self.agent])
        return [ AgentAt(self.agent, destination),
                 Not(AgentAt(self.agent, state.agent_locations[self.agent]))]

    def __lt__(self,other):
        if type(other)==NoOp:
            return False
        return True

    def __repr__(self):
        return 'Move({})'.format(self.direction)
    
    
class MoveActionRobot(Action):

    def __init__(self, state, agent, direction):
        self.agent = agent
        self.direction = direction
        self.state = state
        self.destination = [direction.move((x,y)) for (x,y) in self.state.agent_locations[self.agent]]

    def preconditions(self,state):
        destination = [self.direction.move((x,y)) for (x,y) in state.agent_locations[self.agent]]
        return {Free((x,y)) for (x,y) in destination}

    def effects(self,state):
        destination = [self.direction.move((x,y)) for (x,y) in state.agent_locations[self.agent]]
        effects = [AgentAt(self.agent, (x,y)) for (x,y) in destination] + [Not(AgentAt(self.agent, (x,y))) for (x,y) in state.agent_locations[self.agent]]
        return effects

    def __lt__(self,other):
        if type(other)==NoOp:
            return False
        return True

    def __repr__(self):
        return 'Move({})'.format(self.direction)

    
class Enter(Action):
    
    def __init__(self, agent):
        self.agent = agent
        self.destination = agent.start_pos
        
    def preconditions(self,state):
        return {Free(self.destination), AgentAt(self.agent, (-1,-1))}
    
    def effects(self, state):
        return [AgentAt(self.agent, self.destination), Not(AgentAt(self.agent, (-1,-1)))]
    
    def __lt__(self,other):
        if type(other)==NoOp:
            return False
        return True

    def __repr__(self):
        return 'Enter({})'.format(self.agent)
    
    
    
class Leave(Action):
    
    def __init__(self, agent):
        self.agent = agent
        self.destination = (-1,-1)
        
    def preconditions(self):
        return {AgentAt(self.agent, self.agent.goal)}
    
    def effects(self, state):
        return [Not(AgentAt(self.agent, state.agent_locations[self.agent])), AgentAt(self.agent, (-1,-1))]
    
    def __lt__(self,other):
        if type(other)==NoOp:
            return False
        return True

    def __repr__(self):
        return 'Leave({})'.format(self.agent)
    
    
    
    
    
    
    
    
    
    
    