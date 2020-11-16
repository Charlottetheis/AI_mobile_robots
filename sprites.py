import pygame
from actions import *
import random



class wall_sprite(pygame.sprite.Sprite):
    def __init__(self,x, y):
       # Call the parent class (Sprite) constructor
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.Surface([16,16])
        self.image.fill((150,150,150))
        self.rect = self.image.get_rect()
        self.rect = self.rect.move(x,y)
        
        
        
class human_sprite(pygame.sprite.Sprite):

    # Constructor. Pass in the color of the block,
    # and its x and y position
    def __init__(self, plan, start_pos, start_time):
       # Call the parent class (Sprite) constructor
        pygame.sprite.Sprite.__init__(self)

       # Create an image of the block, and fill it with a color.
       # This could also be an image loaded from the disk.
        #self.image = pygame.Surface([16, 16])
        #self.image.fill((255,255,255))
        
        n = random.randint(1,3)
        self.image = pygame.image.load("human" + str(n) + ".png")

       # Fetch the rectangle object that has the dimensions of the image
       # Update the position of this object by setting the values of rect.x and rect.y
        
        self.start = start_time
        self.start_pos = start_pos
        self.rect = self.image.get_rect().move([self.start_pos[0]*16,self.start_pos[1]*16])
        self.plan = plan
        self.speed = [0,0]
    
    def get_coor(self):
        return [self.rect[0] % width, self.rect[1] % height]
    
    def act(self):
        
        action = self.plan.pop()
        if type(action) == MoveAction:
            direction = action.direction.move((0,0))
            self.speed = list(direction)
        else: self.speed = [0,0] 
            
            
class robot_sprite(pygame.sprite.Sprite):

    # Constructor. Pass in the color of the block,
    # and its x and y position
    def __init__(self, plan, start_pos, start_time):
       # Call the parent class (Sprite) constructor
        pygame.sprite.Sprite.__init__(self)

        self.image = pygame.image.load("robot.png")

       # Fetch the rectangle object that has the dimensions of the image
       # Update the position of this object by setting the values of rect.x and rect.y
        
        self.start = start_time
        self.start_pos = start_pos
        self.rect = self.image.get_rect().move([self.start_pos[0]*16,self.start_pos[1]*16])
        self.plan = plan
        self.speed = [0,0]
    
    def get_coor(self):
        return [self.rect[0] % width, self.rect[1] % height]
    
    def act(self):
        
        action = self.plan.pop()
        if type(action) == MoveActionRobot:
            direction = action.direction.move((0,0))
            self.speed = list(direction)
        else: self.speed = [0,0] 
            
            
class bed_sprite(pygame.sprite.Sprite):

    # Constructor. Pass in the color of the block,
    # and its x and y position
    def __init__(self, position):
       # Call the parent class (Sprite) constructor
        pygame.sprite.Sprite.__init__(self)

       # Create an image of the block, and fill it with a color.
       # This could also be an image loaded from the disk.
        #self.image = pygame.Surface([16*2, 16*3])
        #self.image.fill((255,255,255))
        self.image = pygame.image.load("bed.png")

       # Fetch the rectangle object that has the dimensions of the image
       # Update the position of this object by setting the values of rect.x and rect.y
        
        self.position = position
        self.rect = self.image.get_rect().move([self.position[0][0]*16,self.position[0][1]*16])

           
        
        