import networkx as nx
import pygame
from sprites import *

def parse_map(s):
    """
    Parse a level with the format:

    <map>
    """

    # We split the string into lines, and go each line
    lines = s.splitlines()

    # Map the nodes as a graph
    y_dim = len(lines)
    x_dim = len(lines[0])
    G = nx.grid_2d_graph(x_dim,y_dim)
    

    # create wall sprite for visualization
    walls = pygame.sprite.RenderUpdates()
    
    for y, line in enumerate(lines):
        for x, char in enumerate(line):
            if char == 'w':
                G.remove_node((x,y))
                walls.add(wall_sprite(x*16,y*16))
            elif char == 'e':
                G.nodes[(x,y)]['type'] = 'e'
            elif char == 's':
                G.nodes[(x,y)]['type'] = 's'
            elif char == ' ':
                G.nodes[(x,y)]['type'] = 'f'
            elif char == 'b':
                G.nodes[(x,y)]['type'] = 'b'
            elif char == 'r':
                G.nodes[(x,y)]['type'] = 'r'
            else:
                G.nodes[(x,y)]['type'] = 'goal'
                G.nodes[(x,y)]['number'] = int(char)
                
                
                
   # G.add_edges_from([((x, y), (x+1, y+1)) for x in range(x_dim-1) for y in range(y_dim-1) 
    #                  if ((x,y+1) in G.nodes and (x+1,y) in G.nodes and (x,y) in G.nodes and (x+1,y+1) in G.nodes)] 
    #                 + [((x+1, y), (x, y+1)) for x in range(x_dim-1) for y in range(y_dim-1) 
    #                    if ((x,y+1) in G.nodes and (x+1,y) in G.nodes and (x,y) in G.nodes and (x+1,y+1) in G.nodes)], weight=2**0.5)
                

        
    
    return (G, walls)
