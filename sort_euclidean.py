from math import sqrt, pow


def sort_euclidean(start, goals):
    '''
    Calculates the euclidean distance from start to each goal in the list of
    goals, then returns a sorted list of goals from nearest to furthest
    '''
    sorted_goals = []

    for goal in goals:
        x_diff = pow(goal.x - start.x, 2)
        y_diff = pow(goal.y - start.y, 2)
        distance = sqrt(x_diff + y_diff)
        sorted_goals.append(distance)
    sorted_goals.sort()

    return sorted_goals
