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
from online_replanning_robot import *
from itertools import chain



def simulate(m,viz=False):
    G, sprite_walls = parse_map(m)
    
    #starting positions for humans
    start_pos = [(x,y) for (x,y),att in G.nodes('type') if att=='s']
    #entrance and exit positions
    entrance_pos = [(x,y) for (x,y),att in G.nodes('type') if att=='e']
    #robot start positions
    robot_start = [(x,y) for (x,y),att in G.nodes('type') if att=='r']
    #robot goal positions
    robot_goals = [(x,y) for (x,y),att in G.nodes('type') if att=='goal']
    
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
        
        state = State(G, 0, preds, {h: start}, object_locations)
        plan = offline_search(state, h, goal, manhattan(h, goal)) #calculate path to goal
        
        h.plan = [Enter(h)] + plan + [Leave(h)]
        humans.append(h) #human objects
        human_id += 1
    
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
        
        preds = predicates.copy()
        preds.add(AgentAt(h, start))
        state = State(G, 0, preds, {h: start}, object_locations)
        plan = offline_search(state, h, goal, manhattan(h, goal)) #calculate path to goal
        
        h.plan = [Enter(h)] + plan + [Leave(h)]
        humans.append(h) #human objects
        human_id += 1
    
    
    
    
    ############## Initialize robot #############################
    free_predicates = {Free((x,y)) for (x,y),att in G.nodes('type') if att in ['f','e','s', 'b','goal','r']}
    start_pos=robot_start[0]
    r = robot(0, start_pos, start_time=0, offline_plan=[])
    start = start_pos
    for i in range(1,len(robot_goals)+2):
        preds = free_predicates.copy()
        preds.update([AgentAt(r, (x,y)) for (x,y) in r.get_full(start)])
        preds.difference_update([Free((x,y)) for (x,y) in r.get_full(start)])
        state = State(G, 0, preds, {r: r.get_full(start)}, object_locations)
        if i == len(robot_goals)+1:
            goal = r.get_full(start_pos)
        else:
            goal = [coor for coor in robot_goals if G.nodes('number')[coor] == i]
            goal = r.get_full(goal[0])
        plan = offline_search(state, r, goal, manhattan_robot(r, goal)) #calculate path to goal
        r.plan+=plan 
        r.goal.append(set(goal))
        start = goal[0]
    
    #update the permanent offline/global plan
    r.global_path = r.plan_to_path()
    r.global_plan = r.plan.copy()
    
    ############### online part #################################
    not_finished = True
    t = 0
    
    #initialize state
    preds = predicates.copy()
    agent_locations = dict()             
    for h in humans:
        preds.add(AgentAt(h,(-1,-1)))
        agent_locations[h] = (-1,-1)
    preds.update([AgentAt(r, (x,y)) for (x,y) in r.get_full(start_pos)])
    preds.difference_update([Free((x,y)) for (x,y) in r.get_full(start_pos)])
    agent_locations[r] = r.get_full(start_pos)
    state = State(G,0,preds,agent_locations,object_locations)
    
    while not_finished:
        # Finished robot plan
        if len(r.plan) <= t:
            not_finished = False
        else:
            # replanning in case obstacle moved
            if t in r.obstacle:
                obstacle_coors = [Free(x) for x in r.obstacle[t]]
                if all([coor in state.predicates for coor in obstacle_coors]):
                    print('obstacle has moved at time ' + str(t))
                    r, new_actions, back_at_path = replan_robot(state, r, t, \
                                                                global_plan=r.previous_plan)
                    print('New actions robot ' +str(new_actions))
                    print('Back at path robot ' +str(back_at_path))
                    if (new_actions!=None) and ((back_at_path+t)<r.back_at_global_path[t]):
                        new_actions = [action[0] for action in new_actions]
                        # Update the current plan
                        del r.plan[t-r.start_time:]
                        r.plan += new_actions
                        r.plan += r.previous_plan[t+back_at_path:]
                        r.previous_plan = r.plan.copy()
                        
                        
                        
                        #update history
                        for i in range(t,r.detour[t][1]):
                            del r.obstacle[i]
                            del r.detour[i]
                            del r.back_at_global_path[i]
                        r.replanning[t] = r.plan_to_path()
            
            
            else:        
                # Replanning for robots in case of objects in path
                human_loc = [pred.coordinate for pred in state.predicates if (type(pred)==AgentAt) and (pred.agent!=r)]
                object_loc = [pred.coordinate for pred in state.predicates if (type(pred)==ObjectAt)]
                all_loc = human_loc + [coor for coor in chain.from_iterable(object_loc)]
                r_path = r.plan_to_path()
                r_path = r_path[t+1:t+4]
                if any([coor in chain.from_iterable(r_path) for coor in all_loc]):

                    print('Something in robot path at ' + str(t))

                    r, new_actions, back_at_path = replan_robot(state, r, t)
                    if new_actions == None:
                        new_actions = [NoOp(state, r)]*4
                    else:
                        new_actions = [action[0] for action in new_actions]
                    print('New actions robot ' +str(new_actions))
                    print('Back at path robot ' +str(back_at_path))

                    # Update the current plan
                    r.previous_plan = r.plan.copy()
                    del r.plan[t-r.start_time:back_at_path+t-r.start_time]
                    r.plan[t-r.start_time:t-r.start_time] = new_actions

                    #update history
                    obs_coor = [coor for coor in all_loc if (coor in chain.from_iterable(r_path))]
                    for i in range(0,len(new_actions)):
                        r.obstacle[t+i] = obs_coor 
                        r.detour[t+i] = [t,t+len(new_actions)] #[start,finish]
                        r.back_at_global_path[t+i] = t+back_at_path
                    r.replanning[t] = r.plan_to_path()

            #All actions at time t
            actions_t = [h.plan[t-h.start_time] for h in humans if ((len(h.plan)-1+h.start_time) >= t) & (h.start_time <= t)]
            actions_t += [r.plan[t]] 
            
            # Checking consistency
            preconditions = [action.preconditions(state) for action in actions_t]
            destinations = [action.destination for action in actions_t[0:-1] if type(action)!=Leave]
            destinations += actions_t[-1].destination
            print('preconditions: ' + str(preconditions))
            print('does the precond hold: ' + str([precond in state.predicates for precond in chain.from_iterable(preconditions)]))
            print('destinations: ' + str(destinations))
            print('agent locations ' + str(state.agent_locations))
            
            #If consistent update state
            if all([precond in state.predicates for precond in chain.from_iterable(preconditions)]) \
                                & (len(destinations) == len(set(destinations))):
                #print('Time: ' + str(t) + ' no conflicts')
                
                #calculate conflicts three timesteps ahead
                #if there are conflicts: Replan around them 
                
                state = state.derive_state(actions_t)
                t += 1
            
            else:
                # Replanning with two humans or one human+passive robot
                # The robot stps when a human is very close to it
                print('replanning at time: ' + str(t))
                agents = humans + [r]
                agents_replanned, actions, back_at_path = replan(state,agents,t)
                print('agents replanned ' + str(agents_replanned))
                for i, h in enumerate(agents_replanned):
                    if type(h)==robot:
                        h.plan[t-h.start_time:t-h.start_time] = [NoOp(state, h),NoOp(state, h),NoOp(state, h),NoOp(state, h)]
                    else:
                        if actions != None:
                            new_actions = [action[i] for action in actions]

                            del h.plan[t-h.start_time:back_at_path[i]+t-h.start_time]
                        else:
                            new_actions = [NoOp(state, h)]
                        #print('new actions: ' + str(new_actions))
                        #print('new actions preconditions: ' + str([action.preconditions(action.state) for action in new_actions]))
                        #print('back at path: ' + str(back_at_path[i]))
                        h.plan[t-h.start_time:t-h.start_time] = new_actions
  
            
    
    #visualization module
    if viz:
        size = width, height = 526, 480
        screen = pygame.display.set_mode(size) 
        frame = 0
        
        sprite_humans = pygame.sprite.RenderUpdates()
        
        sprite_beds = []
        for b in beds:
            sprite_beds.append(bed_sprite(b.position))
        
        sprite_objects = pygame.sprite.RenderUpdates(sprite_beds)
        
        plan = r.plan.copy()
        plan.reverse()
        sprite_robot = robot_sprite(plan, r.start_pos, r.start_time, r.obstacle)
        sprite_robots = pygame.sprite.RenderUpdates(sprite_robot)
        print(r.obstacle)
        paths = []
        for s in r.global_path:
            [paths.append(path_sprite(coor)) for coor in s]
        sprite_paths = pygame.sprite.RenderUpdates(paths)
            
        sprite_obstacles = pygame.sprite.RenderUpdates()

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
                #move all objects
                h.rect = h.rect.move(h.speed)
            
            for r in sprite_robots:
                if frame%16==0:
                    sprite_obstacles.empty()
                    if r.plan == []:
                        r.speed = [0,0]
                    else:
                        r.act()
                        if (frame/16) in r.obstacle:
                            sprite_obstacles.add([obstacle_sprite(o) for o in r.obstacle[frame/16]])
                            print('obstacle' + str(r.obstacle[frame/16]))
                r.rect = r.rect.move(r.speed)
            
            screen.fill((220,200,200))
            sprite_walls.draw(screen)
            sprite_paths.draw(screen)
            sprite_objects.draw(screen)
            sprite_humans.draw(screen)
            sprite_obstacles.draw(screen)
            sprite_robots.draw(screen)
            
            pygame.font.init()
            myFont = pygame.font.SysFont("Times New Roman", 18)
            Label = myFont.render("Timestep:", 1, (0,0,0))
            ### pass a string to myFont.render
            time = frame/16
            Display = myFont.render(str(time), 1, (0,0,0))

            screen.blit(Label, (10, 20))
            screen.blit(Display, (10, 30))
            
            pygame.time.delay(30)
            pygame.display.update()
            
                        
            frame += 1 #update frame number
                
                
                
                
                