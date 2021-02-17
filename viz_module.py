import networkx as nx
import pygame
from sprites import *
from graph_parser import *
import random
import sys
import math

def visualise(r, humans, beds, sprite_walls, run_no, ex):
    size = width, height = 416, 528
    screen = pygame.display.set_mode(size) 
    frame = 0
    pygame.time.delay(3000)
    sprite_humans = pygame.sprite.RenderUpdates()

    sprite_beds = []
    for b in beds:
        sprite_beds.append(bed_sprite(b.position))

    sprite_objects = pygame.sprite.RenderUpdates(sprite_beds)

    plan = r.plan.copy()
    plan.reverse()
    sprite_robot = robot_sprite(plan, r.start_pos, r.start_time, r.obstacle)
    sprite_robots = pygame.sprite.RenderUpdates(sprite_robot)
    paths = []
    for s in r.global_path:
        [paths.append(path_sprite(coor)) for coor in s]
    sprite_paths = pygame.sprite.RenderUpdates(paths)

    sprite_obstacles = pygame.sprite.RenderUpdates()
    
    running=1
    while running:            
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
                        sprite_obstacles.add([obstacle_sprite(o) for o in r.obstacle[frame/16] if type(o) == tuple])
                        print('obstacle' + str(r.obstacle[frame/16]))
            r.rect = r.rect.move(r.speed)

        screen.fill((211,197,203))
        sprite_walls.draw(screen)
        #sprite_paths.draw(screen)
        sprite_objects.draw(screen)
        sprite_humans.draw(screen)
        #sprite_obstacles.draw(screen)
        sprite_robots.draw(screen)

        pygame.font.init()
        myFont = pygame.font.SysFont("Times New Roman", 18)
        Label = myFont.render("Timestep:", 1, (0,0,0))
        Label2 = myFont.render("Run:", 1, (0,0,0))
        Label3 = myFont.render("Type:", 1, (0,0,0))
        ### pass a string to myFont.render
        time = math.floor(frame/16)
        Display = myFont.render(str(time), 1, (0,0,0))
        Display2 = myFont.render(str(run_no), 1, (0,0,0))
        Display3 = myFont.render(ex, 1, (0,0,0))

        screen.blit(Label, (5, 41))
        screen.blit(Label2, (5, 1))
        screen.blit(Label3, (5, 21))
        screen.blit(Display, (85, 41))
        screen.blit(Display2, (85, 1))
        screen.blit(Display3, (85, 21))

        pygame.time.delay(30)
        pygame.display.update()


        frame += 1 #update frame number

        if r.plan == []:
            running=0
            pygame.display.quit()
            pygame.quit()       
                
                