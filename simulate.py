import networkx as nx
import pygame
from sprites import *
from graph_parser import *
import random
import sys
from objects import *
import numpy as np
from predicates import *
from state import *
from offline_search import *
from actions import *
from online_replanning import *
from itertools import chain



def simulate(m,viz=False):
    G, sprite_walls = parse_map(m)
    
    #starting positions for humans
    start_pos = [(x,y) for (x,y),att in G.nodes('type') if att=='s']
    #entrance and exit positions
    entrance_pos = [(x,y) for (x,y),att in G.nodes('type') if att=='e']
    
    free_predicates = {Free((x,y)) for (x,y),att in G.nodes('type') if att in ['f','e','s']}
    object_locations = dict()
    
    humans = []
    human_id = 0
    #alloctae a random destination to humans
    for start in start_pos:
        preds = free_predicates.copy()
        preds.add(AgentAt(human_id, start))
        
        
        goal = entrance_pos[random.randint(0,len(entrance_pos)-1)] #assign a random goal
        
        h = human(human_id, start_pos=start, start_time=0, goal=goal)
        
        state = State(G, 0, preds, {h: start}, object_locations)
        plan = offline_search(state, h, start, goal, manhattan(h, goal)) #calculate path to goal
        
        h.plan = [Enter(h)] + plan + [Leave(h)]
        humans.append(h) #human objects
        human_id += 1
    
    times_taken = [] #to keep track of no humans entering at the same time
    for i in range(10):
        goal = start = (0,0)
        #the distance between start and end must be more than 5
        while sum(list(map(lambda i, j: abs(j - i), start, goal))) <= 5:
            start = entrance_pos[random.randint(0,len(entrance_pos)-1)]
            goal = entrance_pos[random.randint(0,len(entrance_pos)-1)]
         
        #random time of arrival
        #time = random.randint(0,70)
        #while time in times_taken:
        #    time = random.randint(0,70)
        #times_taken.append(time)
        time = human_id*5
        
        h = human(human_id, start_pos=start, start_time=time, goal=goal)
        
        preds = free_predicates.copy()
        preds.add(AgentAt(h, start))
        state = State(G, 0, preds, {h: start}, object_locations)
        plan = offline_search(state, h, start, goal, manhattan(h, goal)) #calculate path to goal
        
        h.plan = [Enter(h)] + plan + [Leave(h)]
        humans.append(h) #human objects
        human_id += 1
    
    ############### online part #################################
    not_finished = True
    conflict_count = 0
    t = 0
    preds = free_predicates.copy()
    agent_locations = dict()             
    for h in humans:
        preds.add(AgentAt(h,(-1,-1)))
        agent_locations[h] = (-1,-1)
    state = State(G,0,preds,agent_locations,object_locations)
    
    while not_finished:
        actions_t = [h.plan[t-h.start_time] for h in humans if ((len(h.plan)-1+h.start_time) >= t) & (h.start_time <= t)]
        print(actions_t)
        if len(actions_t) == 0:
            not_finished = False
        else:
            preconditions = [action.preconditions() for action in actions_t]
            destinations = [action.destination for action in actions_t if type(action)!=Leave]
            print('preconditions: ' + str(preconditions))
            print('does the precond hold: ' + str([precond in state.predicates for precond in chain.from_iterable(preconditions)]))
            print('destinations: ' + str(destinations))
            
            if all([precond in state.predicates for precond in chain.from_iterable(preconditions)]) \
                                & (len(destinations) == len(set(destinations))):
                print('Time: ' + str(t) + ' no conflicts')
                state = state.derive_state(actions_t)
                t += 1
            else:
                print('replanning at time: ' + str(t))
                humans_replanned, actions, back_at_path = replan(state,humans,t)
                for i, h in enumerate(humans_replanned):
                    new_actions = [action[i] for action in actions]
                    print('new actions: ' + str(new_actions))
                    print('new actions preconditions: ' + str([action.preconditions() for action in new_actions]))
                    print('back at path: ' + str(back_at_path[i]))
                    del h.plan[t-h.start_time:back_at_path[i]+t-h.start_time]
                    h.plan[t-h.start_time:t-h.start_time] = new_actions
                    
                    print('precondition for: ' + str(h) + ' at time: ' + str(t) + ': ' + str(h.plan[t-h.start_time].preconditions()))
            
            
        #for h1 in range(len(resources)):
        #    for h2 in range(h1+1, len(resources)):
        #        if (any([coor in resources[h1] for coor in resources[h2]])):
        #            print(str(h1)+ ' and ' + str(h2) + ' in conflict at time ' + str(t))
                        
    
    
    
    
    #finish_time = 0 
    #for h in humans:
     #   if finish_time < h.end:
     #       finish_time = h.end
            
    #fix the human paths so that they do not overlap
    #resources = []
    #max_len = 0
    #for h in humans:
    #    res = h.get_resources()
    #    resources.append(res)
    #    if len(res) > max_len:
    #        max_len = len(res)
        
   # not_finished = True
   # conflict_count = 0
   # t = 0
   # while not_finished:
   #     for j in range(len(resources)):
   #         for k in range(j+1, len(resources)):
   #             if (len(resources[k])>t) and (len(resources[j])>t) and (any([coor in resources[j][t] for coor in resources[k][t]])):
   #                 conflict_count += 1
   #                 print(str(k)+ ' and ' + str(j) + ' in conflict at time ' + str(t))
   #                 resources[k].insert(t, resources[k][t-1])
   #                 humans[k].plan.insert(t-humans[k].start_time,NoOp(k))
#
   #     if (max_len+conflict_count) < t:
   #         not_finished = False
   #     t +=1
    
    
    #create a group for the sprites
    #sprite_humans = pygame.sprite.RenderUpdates(humans)
    
    
    #visualization module
    if viz:
        size = width, height = 480, 480
        screen = pygame.display.set_mode(size) 
        frame = 0
        
        sprite_humans = pygame.sprite.RenderUpdates()

        while 1:            
            for event in pygame.event.get():
                if event.type == pygame.QUIT: sys.exit()
            
            for h in humans:
                if h.start_time == frame/16:
                    plan = h.plan.copy()
                    plan.reverse()
                    sprite_humans.add(human_sprite(plan,h.start_pos,h.start_time))

            for h in sprite_humans:
                #When the goal is reached (empty plan) the object is removed from the group (e.g. not plotted)
                if h.plan == []:
                    sprite_humans.remove(h)
                    continue
                #Only take an action when the object has reached a new field (16*16 pixels)
                elif frame%16 == 0: #(h.rect.top % 16 == 0) & (h.rect.left % 16 == 0): #
                    h.act()
                    #r.act()
                #move all objects
                h.rect = h.rect.move(h.speed)

            screen.fill((220,200,200))
            sprite_walls.draw(screen)
            sprite_humans.draw(screen)
            
            pygame.time.delay(50)
            pygame.display.update()
            
                        
            frame += 1 #update frame number
                
                
                
                
                