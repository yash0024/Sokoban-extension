from inspect import trace
from itertools import count
import os  # for time functions
import math  # for infinity
import heapq
from search import *  # for search engines
from sokoban import SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

def sokoban_goal_state(state):
    '''
    @return: Whether all boxes are stored.
    '''
    for box in state.boxes:
        if box not in state.storage:
            return False
    return True

def heur_manhattan_distance(state):
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    
    total = 0
    for box in state.boxes:
        total += min([abs(store[0] - box[0]) + abs(store[1] - box[1]) for store in state.storage])
    return total

def heur_alternate(state):
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''

    total = 0
    sortedBoxes = list(state.boxes)
    goals = list(state.storage)
    obs = set(sortedBoxes)
    while sortedBoxes:
        box = sortedBoxes.pop()
        obs.remove(box)
        counts = []
        x1, y1 = box[0], box[1]
        for store in goals:
            x2, y2 = store[0], store[1]
            a, b, c, d = min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)
            counts.append(len([(x, y) for x in range(a, b + 1) for y in range(c, d + 1)]))
        total += min(counts)
        goals.pop(counts.index(min(counts)))
    return total

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return sN.gval + weight * sN.hval

# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
    '''Provides an implementation of weighted a-star.'''
    '''INPUT: a warehouse state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''
    se = SearchEngine('custom', 'full')
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    se.init_search(initial_state, goal_fn = sokoban_goal_state, heur_fn = heur_fn, fval_function = wrapped_fval_function)
    return se.search(timebound)

def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):
    '''Provides an implementation of realtime a-star'''
    '''INPUT: a warehouse state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of realtime astar algorithm'''
    weight = 10
    se = SearchEngine('custom', 'full')
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    se.init_search(initial_state, goal_fn = sokoban_goal_state, heur_fn = heur_fn, fval_function = wrapped_fval_function)
    goal, stats = se.search(timebound)
    if not goal: return False, stats
    else: timebound -= stats.total_time
    while timebound > 0:
        weight /= 2
        wrapped_fval_function = (lambda sN: fval_function(sN, weight))
        se.init_search(initial_state, goal_fn = sokoban_goal_state, heur_fn = heur_fn, fval_function = wrapped_fval_function)
        newgoal, newstats = se.search(timebound, [float('inf'), float('inf'), goal.gval + heur_fn(goal)])
        if not newgoal: 
            break
        timebound -= newstats.total_time
        goal, stats = newgoal, newstats
    return (goal, stats)

def iterative_gbfs(initial_state, heur_fn, timebound=5):
    '''Provides an implementation of anytime greedy best-first search'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''

    se = SearchEngine('best_first', 'full')
    se.init_search(initial_state, goal_fn = sokoban_goal_state, heur_fn = heur_fn)
    goal, stats = se.search(timebound)
    if not goal: return False, stats
    else: timebound -= stats.total_time
    while timebound > 0:
        newgoal, newstats = se.search(timebound, [goal.gval, float('inf'), float('inf')])
        if not newgoal: 
            break
        timebound -= newstats.total_time
        goal, stats = newgoal, newstats
    return (goal, stats) #CHANGE THIS



