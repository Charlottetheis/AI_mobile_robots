from objects import *
from actions import *


def global_costmap(G):
    #Door openings and corners
    for node in G.nodes:
        neigh = G[node]
        if len(neigh) == 3:
            cross_neigh = [tuple(sum(x) for x in zip((1,1),node)),tuple(sum(x) for x in zip((-1,1),node)),\
             tuple(sum(x) for x in zip((1,-1),node)), tuple(sum(x) for x in zip((-1,-1),node))]
            cross_neigh_ex = [n for n in cross_neigh if n in G.nodes]
            if len(cross_neigh_ex) > 2:
                G.nodes[node]['cost'] += 1
        #corners
        if len(neigh) == 4:
            cross_neigh = [tuple(sum(x) for x in zip((1,1),node)),tuple(sum(x) for x in zip((-1,1),node)),\
             tuple(sum(x) for x in zip((1,-1),node)), tuple(sum(x) for x in zip((-1,-1),node))]
            cross_neigh_ex = [n for n in cross_neigh if n in G.nodes]
            if len(cross_neigh_ex) == 3:
                G.nodes[node]['cost'] += 1
    return G
    
def local_costmap(state, r, t, G):
    #Dyrt rundt om agenter
    for agent, time in r.predictions:
        if (time==(t)):
            coor = r.predictions[(agent, time)]
            direction = r.dir_predictions[(agent, time)]
            G.nodes[coor]['cost'] += 92
            for c in G[coor]: #increase all neighbours by 2
                G.nodes[c]['cost'] += 4
            coor_neigh = []
            if direction == 0: #north
                coor_neigh = [tuple(sum(x) for x in zip((1,1),coor)), tuple(sum(x) for x in zip((0,-1),coor)), \
                              tuple(sum(x) for x in zip((1,0),coor)), tuple(sum(x) for x in zip((0,1),coor)), \
                              tuple(sum(x) for x in zip((2,1),coor)), tuple(sum(x) for x in zip((0,-2),coor)),\
                              tuple(sum(x) for x in zip((-1,-1),coor))]
            elif direction == 1: #south
                coor_neigh = [tuple(sum(x) for x in zip((-1,-1),coor)), tuple(sum(x) for x in zip((0,1),coor)), \
                              tuple(sum(x) for x in zip((-1,0),coor)), tuple(sum(x) for x in zip((0,-1),coor)), \
                              tuple(sum(x) for x in zip((-2,-1),coor)), tuple(sum(x) for x in zip((0,2),coor)), \
                              tuple(sum(x) for x in zip((1,1),coor))]
            elif direction == 2: #east
                coor_neigh = [tuple(sum(x) for x in zip((-1,1),coor)), tuple(sum(x) for x in zip((1,0),coor)), \
                              tuple(sum(x) for x in zip((0,1),coor)), tuple(sum(x) for x in zip((-1,0),coor)), \
                              tuple(sum(x) for x in zip((-1,2),coor)), tuple(sum(x) for x in zip((2,0),coor)), \
                              tuple(sum(x) for x in zip((1,-1),coor))]
            elif direction == 3: #west
                coor_neigh = [tuple(sum(x) for x in zip((1,-1),coor)), tuple(sum(x) for x in zip((-1,0),coor)), \
                              tuple(sum(x) for x in zip((0,-1),coor)), tuple(sum(x) for x in zip((1,0),coor)), \
                              tuple(sum(x) for x in zip((1,-2),coor)), tuple(sum(x) for x in zip((-2,0),coor)), \
                              tuple(sum(x) for x in zip((-1,1),coor))]
            
            for i, c in enumerate(coor_neigh):
                if c in G.nodes:
                    #if i == 0:
                    #    G.nodes[c]['cost'] += 4
                    if i in [1, 2]:
                        G.nodes[c]['cost'] += 4
                    elif i == 3:
                        G.nodes[c]['cost'] += 96
                    elif i == 4:
                        G.nodes[c]['cost'] += 4
                    elif i == 5:
                        G.nodes[c]['cost'] += 4
                    elif i == 6:
                        G.nodes[c]['cost'] += 4
                    if i in [2, 3]:
                        for c2 in G[coor_neigh[i]]: #neighbour on the right side
                            G.nodes[c2]['cost'] += 4


    #Dyrt at gå i venstre side af gangen

    #coordinates
    #w  12 w
    #w 8rr3w
    #w 7rr4w
    #w  65 W

    position = state.agent_locations[r]
    x_coor = {coor[0] for coor in position}
    y_coor = {coor[1] for coor in position}
    coor_1 = (min(x_coor), min(y_coor)-1) 
    coor_2 = (max(x_coor), min(y_coor)-1)
    coor_3 = (max(x_coor)+1, min(y_coor))
    coor_4 = (max(x_coor)+1, max(y_coor))
    coor_5 = (max(x_coor), max(y_coor)+1)
    coor_6 = (min(x_coor), max(y_coor)+1)
    coor_7 = (min(x_coor)-1, min(y_coor))
    coor_8 = (min(x_coor)-1, max(y_coor))

    if ((coor_1 not in G.nodes) and (coor_2 not in G.nodes)) or (all([len(G[c]) == 3 for c in [coor_1,coor_2] if c in G.nodes])):
            if (coor_3 in G.nodes) and (coor_4 in G.nodes):
                G.nodes[coor_3]['cost'] += 1
                G.nodes[coor_4]['cost'] += 1
    if ((coor_3 not in G.nodes) and (coor_4 not in G.nodes)) or (all([len(G[c]) == 3 for c in [coor_3,coor_4] if c in G.nodes])):
            if (coor_5 in G.nodes) and (coor_6 in G.nodes):
                G.nodes[coor_5]['cost'] += 1
                G.nodes[coor_6]['cost'] += 1
    if ((coor_5 not in G.nodes) and (coor_6 not in G.nodes)) or (all([len(G[c]) == 3 for c in [coor_5,coor_6] if c in G.nodes])):
            if (coor_7 in G.nodes) and (coor_8 in G.nodes):
                G.nodes[coor_7]['cost'] += 1
                G.nodes[coor_8]['cost'] += 1
    if ((coor_7 not in G.nodes) and (coor_8 not in G.nodes)) or (all([len(G[c]) == 3 for c in [coor_7,coor_8] if c in G.nodes])):
            if (coor_1 in G.nodes) and (coor_2 in G.nodes):
                G.nodes[coor_1]['cost'] += 1
                G.nodes[coor_2]['cost'] += 1

    #Dyrt at gå venstre om en person        


    return G



def local_costmap_human(state, h, t, G):
    
     #coordinates
    #w  1  w
    #w 4h2 w
    #w  3  w
    #w     W
    position = state.agent_locations[h]
    x_coor = position[0]
    y_coor = position[1]
    
    coor_1 = (x_coor, y_coor-1)
    coor_2 = (x_coor+1, y_coor)
    coor_3 = (x_coor, y_coor+1)
    coor_4 = (x_coor-1, y_coor)
    
    if (coor_1 not in G.nodes):
        if coor_2 in G.nodes:
            G.nodes[coor_2]['cost'] += 2
    elif (len(G[coor_1])==3):
        if coor_2 in G.nodes:
            G.nodes[coor_2]['cost'] += 1
            
    if (coor_2 not in G.nodes):
        if coor_3 in G.nodes:
            G.nodes[coor_3]['cost'] += 2
    elif(len(G[coor_2])==3):
        if coor_3 in G.nodes:
            G.nodes[coor_3]['cost'] += 1
    
    if (coor_3 not in G.nodes):
        if coor_4 in G.nodes:
            G.nodes[coor_4]['cost'] += 2
    elif(len(G[coor_3])==3):
        if coor_4 in G.nodes:
            G.nodes[coor_4]['cost'] += 1
            
    if (coor_4 not in G.nodes):
        if coor_1 in G.nodes:
            G.nodes[coor_1]['cost'] += 2
    elif (len(G[coor_4])==3):
        if coor_1 in G.nodes:
            G.nodes[coor_1]['cost'] += 1
    
    return G
            
            
    
    
    
    
    