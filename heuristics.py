import math

def manhattan(agent, goal):
    def h(state):
        location = state.agent_locations[agent]
        return abs(location[0]-goal[0]) + abs(location[1]-goal[1])
    return h

def euclidean_dist(location, goal):
    return math.sqrt((location[0]-goal[0])**2 + (location[1]-goal[1])**2)

def manhattan_robot(agent, goal):
    def h(state):
        location = state.agent_locations[agent]
        x_min = min({l[0] for l in location})
        y_min = min({l[1] for l in location})
        return abs(x_min-goal[0]) + abs(y_min-goal[1])
    return h

def calculate_manhattan_sum(starts, goals):
    dists = [abs(starts[agent][0]-goals[agent][0]) + abs(starts[agent][1]-goals[agent][1]) for agent in range(len(goals))]
    return sum(dists)

def manhattan_sum(agents):
    def h(state):
        starts = [state.agent_locations[agent] for agent in agents]
        goals = [agent.goal for agent in agents]
        return calculate_manhattan_sum(starts, goals)
    return h

def manhattan_to_path(agents,paths):
    def h(state):
        starts = [state.agent_locations[agent] for agent in agents]
        total_dist = 0
        for i in range(len(agents)):
            start = state.agent_locations[agents[i]]
            min_dist = 10000
            for goal in paths[i]:
                dist = abs(start[0]-goal[0]) + abs(start[1]-goal[1])
                if dist < min_dist:
                    min_dist = dist
            total_dist += min_dist
        return total_dist
    return h

def manhattan_to_path_robot(robot,path):
    def h(state):
        start = state.agent_locations[robot]
        x_min = min({l[0] for l in start})
        y_min = min({l[1] for l in start})
        min_dist = 10000
        for goal in path+robot.goal:
            x_min_goal = min({l[0] for l in goal})
            y_min_goal = min({l[1] for l in goal})
            dist = abs(x_min-x_min_goal) + abs(y_min-y_min_goal)
            if dist < min_dist:
                min_dist = dist
        return min_dist
    return h

