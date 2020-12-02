#!/usr/bin/env python
from math import sqrt, pow



########### BASIC HEURISTICS ###########
def euclidean_sort(start, goals_list):
    '''
    Calculates the euclidean distance from start to each goal in the list of
    goals, then returns a sorted list of goals from nearest to farthest
    '''
    sorted_goals = []
    dist_dict = {}

    for i in range(len(goals_list)):
        x_diff = pow(goals_list[i].x - start.x, 2)
        y_diff = pow(goals_list[i].y - start.y, 2)
        distance = sqrt(x_diff + y_diff)
        dist_dict[i] = distance

    sort_dist_dict = sorted(dist_dict.items(), key=lambda x: x[1])
    for goal in sort_dist_dict:
        sorted_goals.append(goals_list[goal[0]])

    return sorted_goals


def manhattan_sort(start, goals_list):
    '''
    Calculates the manhattan distance from start to each goal in the list of
    goals, then returns a sorted list of goals from nearest to farthest
    '''
    sorted_goals = []
    dist_dict = {}

    for i in range(len(goals_list)):
        x_diff = abs(goals_list[i].x - start.x)
        y_diff = abs(goals_list[i].y - start.y)
        distance = x_diff + y_diff
        dist_dict[i] = distance
    
    sort_dist_dict = sorted(dist_dict.items(), key=lambda x: x[1])

    for goal in sort_dist_dict:
        sorted_goals.append(goals_list[goal[0]])

    return sorted_goals


def minkowski_sort(start, goals_list, p=3):
    '''
    Calculates the minkowski distance from start to each goal in the list of
    goals, then returns a sorted list of goals from nearest to farthest. The
    order of the norm is denoted by p
    '''
    sorted_goals = []
    dist_dict = {}

    for i in range(len(goals_list)):
        x_diff = pow(abs(goals_list[i].x - start.x), p)
        y_diff = pow(abs(goals_list[i].y - start.y), p)
        distance = sqrt(x_diff + y_diff)
        dist_dict[i] = distance

    sort_dist_dict = sorted(dist_dict.items(), key=lambda x: x[1])

    for goal in sort_dist_dict:
        sorted_goals.append(goals_list[goal[0]])

    return sorted_goals


def chebyshev_sort(start, goals_list):
    '''
    Calculates the chebyshev distance from start to each goal in the list of
    goals, then returns a sorted list of goals from nearest to farthest.
    '''
    sorted_goals = []
    dist_dict = {}

    for i in range(len(goals_list)):
        x_diff = abs(goals_list[i].x - start.x)
        y_diff = abs(goals_list[i].y - start.y)
        distance = max(x_diff , y_diff)
        dist_dict[i] = distance

    sort_dist_dict = sorted(dist_dict.items(), key=lambda x: x[1])

    for goal in sort_dist_dict:
        sorted_goals.append(goals_list[goal[0]])

    return sorted_goals


def cosine_sort(start, goals_list):
    '''
    Calculates the cosine distance from start to each goal in the list of
    goals, then returns a sorted list of goals from nearest to farthest.
    The start parameter cannot be the origin
    '''
    sorted_goals = []
    dist_dict = {}

    s_norm = sqrt(pow(start.x, 2) + pow(start.y, 2))
    for i in range(len(goals_list)):
        dot = (goals_list[i].x * start.x) + (goals_list[i].y * start.y)
        g_norm = sqrt(pow(goals_list[i].x, 2) + pow(goals_list[i].y, 2))
        if (s_norm * g_norm == 0):
            distance = 0
        else:
            distance = dot / (s_norm * g_norm)
        dist_dict[i] = distance
    
    sort_dist_dict = sorted(dist_dict.items(), key=lambda x: x[1])

    for goal in sort_dist_dict:
        sorted_goals.append(goals_list[goal[0]])

    return sorted_goals


########### DYNAMIC HEURISTICS ###########
def dynamic_sort(name, start, goals_list):
    '''
    Calculates the euclidean distance for each goal to each
    remaining goal to find the closest one, then sorts the
    goals based off this measure and returns the sorted list
    '''
    sorted_goals = []
    
    next_goal = dynamic_helper(name, start, goals_list)
    sorted_goals.append(next_goal)

    while len(goals_list) > 1:
        visited_idx = goals_list.index(next_goal)
        goals_list.remove(goals_list[visited_idx])
        next_goal = dynamic_helper(name, next_goal, goals_list)
        sorted_goals.append(next_goal)
    
    return sorted_goals


def dynamic_helper(name, start, goals_list):
    '''
    Flags the corresponding helper function containing the desired distance 
    calculation method to use 
    '''
    if name == 'euclidean':
        return dynamic_euclidean_helper(start, goals_list)
    elif name == 'manhattan':
        return dynamic_manhattan_helper(start, goals_list)
    elif name == 'minkowski':
        return dynamic_minkowski_helper(start, goals_list)
    elif name == 'chebyshev':
        return dynamic_chebyshev_helper(start, goals_list)
    elif name == 'cosine':
        return dynamic_cosine_helper(start, goals_list)
    

def dynamic_euclidean_helper(curr, goals_list):
    '''
    Helper function for dynamic_euclidean_sort(). Returns the
    closest goal to the current point
    '''
    sorted_goals = []
    dist_dict = {}

    for i in range(len(goals_list)):
        x_diff = pow(goals_list[i].x - curr.x, 2)
        y_diff = pow(goals_list[i].y - curr.y, 2)
        distance = sqrt(x_diff + y_diff)
        dist_dict[i] = distance

    sort_dist_dict = sorted(dist_dict.items(), key=lambda x: x[1])
    for goal in sort_dist_dict:
        sorted_goals.append(goals_list[goal[0]])

    return sorted_goals[0]


def dynamic_manhattan_helper(curr, goals_list):
    '''
    Helper function for dynamic_manhattan_sort(). Returns the
    closest goal to the current point
    '''
    sorted_goals = []
    dist_dict = {}

    for i in range(len(goals_list)):
        x_diff = abs(goals_list[i].x - curr.x)
        y_diff = abs(goals_list[i].y - curr.y)
        distance = x_diff + y_diff
        dist_dict[i] = distance
    
    sort_dist_dict = sorted(dist_dict.items(), key=lambda x: x[1])

    for goal in sort_dist_dict:
        sorted_goals.append(goals_list[goal[0]])

    return sorted_goals[0]


def dynamic_minkowski_helper(curr, goals_list, p=3):
    '''
    Helper function for dynamic_minkowski_sort(). Returns the
    closest goal to the current point
    '''
    sorted_goals = []
    dist_dict = {}

    for i in range(len(goals_list)):
        x_diff = pow(abs(goals_list[i].x - curr.x), p)
        y_diff = pow(abs(goals_list[i].y - curr.y), p)
        distance = sqrt(x_diff + y_diff)
        dist_dict[i] = distance

    sort_dist_dict = sorted(dist_dict.items(), key=lambda x: x[1])

    for goal in sort_dist_dict:
        sorted_goals.append(goals_list[goal[0]])

    return sorted_goals[0]


def dynamic_chebyshev_helper(curr, goals_list):
    '''
    Helper function for dynamic_chebyshev_sort(). Returns the
    closest goal to the current point
    '''
    sorted_goals = []
    dist_dict = {}

    for i in range(len(goals_list)):
        x_diff = abs(goals_list[i].x - curr.x)
        y_diff = abs(goals_list[i].y - curr.y)
        distance = max(x_diff , y_diff)
        dist_dict[i] = distance

    sort_dist_dict = sorted(dist_dict.items(), key=lambda x: x[1])

    for goal in sort_dist_dict:
        sorted_goals.append(goals_list[goal[0]])

    return sorted_goals[0]


def dynamic_cosine_helper(curr, goals_list):
    '''
    Helper function for dynamic_cosine_sort(). Returns the
    closest goal to the current point
    '''
    sorted_goals = []
    dist_dict = {}

    s_norm = sqrt(pow(curr.x, 2) + pow(curr.y, 2))
    for i in range(len(goals_list)):
        dot = (goals_list[i].x * curr.x) + (goals_list[i].y * curr.y)
        g_norm = sqrt(pow(goals_list[i].x, 2) + pow(goals_list[i].y, 2))
        if (s_norm * g_norm == 0):
            distance = 0
        else:
            distance = dot / (s_norm * g_norm)
        dist_dict[i] = distance
    
    sort_dist_dict = sorted(dist_dict.items(), key=lambda x: x[1])

    for goal in sort_dist_dict:
        sorted_goals.append(goals_list[goal[0]])

    return sorted_goals[0]


########### HEURISTIC SELECTOR ###########
def heuristic(name, start, goals_list, dynamic_name='euclidean'):
    '''
    Flags the corresponding heuristic implmentation to use based on the first paramter.
    If the desired heuristic is 'dynamic' the desired dynamic distance method is chosen
    by providing a fourth paramter - the default is 'euclidean'
    '''
    if name == 'euclidean':
        return euclidean_sort(start, goals_list)
    elif name == 'manhattan':
        return manhattan_sort(start, goals_list)
    elif name == 'minkowski':
        return minkowski_sort(start, goals_list)
    elif name == 'chebyshev':
        return chebyshev_sort(start, goals_list)
    elif name == 'cosine':
        return cosine_sort(start, goals_list)
    elif name == 'dynamic':
        return dynamic_sort(dynamic_name, start, goals_list)
