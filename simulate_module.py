import networkx as nx
import pygame
from sprites import *
from graph_parser import *
import random
import sys

def simulate(m,viz=True):
    G, sprite_walls = parse_map(m)
    
    #starting positions for humans
    start_pos = [(x,y) for (x,y),att in G.nodes('type') if att=='s']
    #entrance and exit positions
    entrance_pos = [(x,y) for (x,y),att in G.nodes('type') if att=='e']

    humans = []
    #alloctae a random destination to humans
    for start in start_pos:
        end = entrance_pos[random.randint(0,len(entrance_pos)-1)] #assign a random goal
        path = nx.astar_path(G, start, end) #calculate path to goal
        path.reverse()
        humans.append(human(path, 0)) #human sprites
    
    late_humans = {}
    for i in range(10):
        end = start = (0,0)
        #the distance between start and end must be more than 5
        while sum(list(map(lambda i, j: abs(j - i), start, end))) <= 5:
            start = entrance_pos[random.randint(0,len(entrance_pos)-1)]
            end = entrance_pos[random.randint(0,len(entrance_pos)-1)]
        path = nx.astar_path(G, start, end) #calculate path to goal
        path.reverse()
        #random time of arrival
        time = random.randint(0,70)
        if time in late_humans.keys():
            late_humans[time] = late_humans[time] + [human(path, time)]
        else:
            late_humans[time] = [human(path, time)] #human sprites
        

    #create a group for the sprites
    sprite_humans = pygame.sprite.RenderUpdates(humans)
    
    
    #visualization module
    if viz:
        size = width, height = 480, 480
        screen = pygame.display.set_mode(size) 
        frame = 0
        while 1:            
            for event in pygame.event.get():
                if event.type == pygame.QUIT: sys.exit()

            if frame/16 in late_humans.keys():
                sprite_humans.add(late_humans[frame/16])
                
            for h in sprite_humans:
                #When the goal is reached (empty plan) the object is removed from the group (e.g. not plotted)
                if h.plan == []:
                    sprite_humans.remove(h)
                else:
                    #Only take an action when the object has reached a new field (16*1 pixels)
                    if (h.rect.top % 16 == 0) & (h.rect.left % 16 == 0): #frame%16 == 0:
                        #if collision(h1, r):
                        ##    h1.replan()
                        #    r.replan()
                       # else:
                        h.act()
                        #r.act()
                    if h.time_to_next==23:
                        if frame%3 < 2:
                            h.rect = h.rect.move(h.speed)
                    else:
                        h.rect = h.rect.move(h.speed)

            screen.fill((220,200,200))
            sprite_walls.draw(screen)
            sprite_humans.draw(screen)
            
            pygame.time.delay(50)
            pygame.display.update()
            
                        
            frame += 1 #update frame number
                
                
                
                
                