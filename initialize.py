from itertools import product, chain
from predicates import *
from state import *
from queue import PriorityQueue
import copy
import networkx as nx
import pygame
from sprites import *
from graph_parser import *
import random
import sys
from objects import *
import numpy as np
from offline_search import *

def human_init(G, h_n):
    #starting positions for humans
    start_pos = [(x,y) for (x,y),att in G.nodes('type') if att=='s']
    #entrance and exit positions
    entrance_pos = [(x,y) for (x,y),att in G.nodes('type') if att=='e']
    
    predicates = {Free((x,y)) for (x,y),att in G.nodes('type') if att in ['f','e','s', 'b','goal','r']}
    
    bed_locations = [(x,y) for (x,y),att in G.nodes('type') if att=='b']
    object_locations = dict()
    bed_id = 0
    beds = []
    #placing random obstacles
    if len(bed_locations) > 0:
        for i in range(20):
            x,y = bed_locations[random.randint(0,len(bed_locations)-1)]
            bed_position = [(x,y),(x+1,y),(x,y+1),(x+1,y+1),(x,y+2),(x+1,y+2)]
            if all([Free(coor) in predicates for coor in bed_position]):
                predicates.difference_update([Free(coor) for coor in bed_position])
                b = bed(bed_id, bed_position)
                predicates.add(ObjectAt(b, bed_position))
                object_locations[b] = bed_position
                beds.append(b)
                bed_id += 1   
    
    humans = []
    human_id = 0
    #alloctae a random destination to humans
    for start in start_pos:
        goal = entrance_pos[random.randint(0,len(entrance_pos)-1)] #assign a random goal
        
        h = human(human_id, start_pos=start, start_time=0, goal=goal)
                
        preds = predicates.copy()
        preds.add(AgentAt(h, start))
        
        state = State(G, 0, 0, preds, {h: start}, object_locations)
        plan = offline_search(state, h, goal, manhattan(h, goal), G) #calculate path to goal
        
        h.plan = [Enter(h)] + plan + [Leave(h)]
        humans.append(h) #human objects
        human_id += 1
    
    for i in range(h_n):
        goal = start = (0,0)
        #the distance between start and end must be more than 5
        while sum(list(map(lambda i, j: abs(j - i), start, goal))) <= 5:
            start = entrance_pos[random.randint(0,len(entrance_pos)-1)]
            goal = entrance_pos[random.randint(0,len(entrance_pos)-1)]
         
        time = human_id*6
        
        h = human(human_id, start_pos=start, start_time=time, goal=goal)
        
        preds = predicates.copy()
        preds.add(AgentAt(h, start))
        state = State(G, 0, 0, preds, {h: start}, object_locations)
        plan = offline_search(state, h, goal, manhattan(h, goal), G) #calculate path to goal
        
        h.plan = [Enter(h)] + plan + [Leave(h)]
        humans.append(h) #human objects
        human_id += 1
   
    return humans, beds, predicates, object_locations

def robot_init(G, costmap):
    #robot start positions
    robot_start = [(x,y) for (x,y),att in G.nodes('type') if att=='r']
    #robot goal positions
    robot_goals = [(x,y) for (x,y),att in G.nodes('type') if att=='goal']
    
    ############## Initialize robot #############################
    free_predicates = {Free((x,y)) for (x,y),att in G.nodes('type') if att in ['f','e','s', 'b','goal','r']}
    start_pos=robot_start[0]
    r = robot(0, start_pos, start_time=0, offline_plan=[])
    start = start_pos
    for i in range(1,len(robot_goals)+2):
        preds = free_predicates.copy()
        preds.update([AgentAt(r, (x,y)) for (x,y) in r.get_full(start)])
        preds.difference_update([Free((x,y)) for (x,y) in r.get_full(start)])
        state = State(G, 0, 0, preds, {r: r.get_full(start)},{})
        if i == len(robot_goals)+1:
            goal = r.get_full(start_pos)
        else:
            goal = [coor for coor in robot_goals if G.nodes('number')[coor] == i]
            goal = r.get_full(goal[0])
        
        plan = offline_search(state, r, goal, manhattan_robot(r, goal[0]), costmap) #calculate path to goal
        r.plan+=plan 
        r.goal.append(set(goal))
        start = goal[0]
    
    #update the permanent offline/global plan
    r.global_path = r.plan_to_path()
    r.global_plan = r.plan.copy()
    
    return r, start_pos
