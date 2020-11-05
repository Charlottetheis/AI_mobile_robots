def manhattan(agent, goal):
    def h(state):
        location = state.agent_locations[agent]
        return abs(location[0]-goal[0]) + abs(location[1]-goal[1])
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