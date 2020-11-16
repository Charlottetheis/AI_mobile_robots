import numpy as np
from actions import *
        
        
class human():
    def __init__(self, human_id, start_pos, start_time, goal=None, offline_plan=[]):
        self.human_id = human_id
        self.start_time = start_time
        self.start_pos = start_pos
        self.plan = offline_plan
        self.goal = goal
        
    
    def get_resources(self, state, time):
        #zero_res = [[] for i in range(self.start_time)]
        #res = [[action.state.agent_locations[self.human_id]] + \
        #                 [action.direction.move(action.state.agent_locations[self.human_id])] for action in self.plan]
        #return zero_res + res
        if self.start_time > time:
            return []
        elif (len(self.plan)-1+self.start_time) < time:
            return []
        else: 
            action = self.plan[time-self.start_time]
            resources = [state.agent_locations[self]]
            if type(action) == MoveAction:
                resources.append(action.direction.move(state.agent_locations[self]))
            elif type(action) == Enter:
                resources = [self.start_pos]
            return resources
       
    def plan_to_path(self):
        position = (-1,-1)
        path = []
        for time in range(len(self.plan)): 
            path.append(position)
            if type(self.plan[time]) == NoOp:
                continue
            else:
                position = self.plan[time].destination
        path.append((-1,-1))
        return path
       
    
    def __repr__(self):
        return 'human ' + str(self.human_id)
    
    
    
    
    
    
class robot():
    def __init__(self, robot_id, start_pos, start_time, goal=None, offline_plan=[]):
        self.id = robot_id
        self.start_time = start_time
        self.start_pos = start_pos
        self.plan = offline_plan
        self.goal = goal
        
    
    def get_resources(self, state, time):
        #zero_res = [[] for i in range(self.start_time)]
        #res = [[action.state.agent_locations[self.human_id]] + \
        #                 [action.direction.move(action.state.agent_locations[self.human_id])] for action in self.plan]
        #return zero_res + res
        if self.start_time > time:
            return []
        elif (len(self.plan)-1+self.start_time) < time:
            return []
        else: 
            action = self.plan[time-self.start_time]
            resources = state.agent_locations[self]
            if type(action) == MoveAction:
                resources.append(self.get_full(action.direction.move(state.agent_locations[self][0])))
            elif type(action) == Enter:
                resources = [self.get_full(self.start_pos)]
            return resources
       
    def plan_to_path(self):
        path = []
        position = []
        for time in range(len(self.plan)): 
            if type(self.plan[time]) == NoOp:
                continue
            else:
                position = self.plan[time].destination
            path.append(position)
        return path
    
    def get_full(self, coor):
        return [coor,tuple(sum(x) for x in zip((1,0),coor)),\
                tuple(sum(x) for x in zip((0,1),coor)),tuple(sum(x) for x in zip((1,1),coor))]
    
    
    def __repr__(self):
        return 'Robot ' + str(self.id)
    
    
    
    
class bed():
    def __init__(self, bed_id, position):
        self.id = bed_id
        self.position = position
        
    def __repr__(self):
        return 'Bed ' + str(self.id)
        
        
        
        
        
        
    
    
    
    