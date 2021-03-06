#!/usr/bin/env python
# astar implementation needs to go here
import math
import numpy as np

def astar_path(grid, start, goal, walls, pits, move_list):
  explored = []
  paths = []
  paths.append([start])
  while True:
    paths_list = [getTotalCost(p,goal) for p in paths]
    mypath =  paths[paths_list.index(min(paths_list))][:]
    neighbors = find_valid_neighbor(mypath[-1],walls, pits, explored, grid)
    for n in neighbors:
        new_path = mypath[:]
        new_path.append(n)
        paths.append(new_path)
        explored.append(n)
    paths.remove(mypath)
    for p in paths:
        if p[-1] == goal:
            return p

def getTotalCost(path, goal):
    if len(path) == 0:
        return 0
    uniCost = len(path) - 1
    #heuristic = euclidean_heuristic_cost(path[-1], goal)
    heuristic = manhattan_heuristic_cost(path[-1], goal)
    #heuristic = chebyshev_heuristic_cost(path[-1], goal)
    #heuristic = octile_heuristic_cost(path[-1], goal)
    return (uniCost + heuristic)

def find_valid_neighbor(pos, walls, pits, explored, grid):
    neighbor_list = []
    #move_list = [[0, 1], [0, -1], [1, 0], [-1, 0]]
    move_list_8 = [[0, 1], [0, -1], [1, 0], [-1, 0], [-1, 1], [1, 1], [-1, -1], [1, -1]]
    for m in move_list_8:
    #for m in move_list:
        neigh_x = pos[0] + m[0]
        neigh_y = pos[1] + m[1]
        temp1 = [neigh_x, neigh_y]
	
        if is_valid_move(temp1, walls, pits, explored, grid):
            neighbor_list.append(temp1)
    return neighbor_list

def find_neighbor(pos):
  neighbor_list = []
  #move_list = [[0,1], [0,-1], [1,0], [-1,0]]
  move_list_8 = [[0, 1], [0, -1], [1, 0], [-1, 0], [-1, 1], [1, 1], [-1, -1], [1, -1]]
  for m in move_list_8:
  #for m in move_list:
    neigh_x = pos[0] + m[0]
    neigh_y = pos[1] + m[1]
    temp1 = [neigh_x, neigh_y]
    neighbor_list.append(temp1)
  return neighbor_list

def is_neighbour(pos1, pos2):
  #move_list = [[0,1], [0,-1], [1,0], [-1,0]]
  move_list_8 = [[0, 1], [0, -1], [1, 0], [-1, 0], [-1, 1], [1, 1], [-1, -1], [1, -1]]
  for m in move_list_8:
  #for m in move_list:
    if (pos1[0] + m[0] == pos2[0]) and (pos1[1] + m[1] == pos2[1]):
      return True
  return False

def is_valid_move(move, walls, pits, explored, grid):
  if move in walls or move in pits or move in explored or (move[0] > len(grid)) or (move[0] < 0) or (move[1] > len(grid[0])) or (move[1] <0):
    return False
  return True

def euclidean_heuristic_cost(pos1, pos2):
  dx = abs(pos1[0] - pos2[0])
  dy = abs(pos1[1] - pos2[1])
  return math.sqrt(dx*dx + dy*dy)

def manhattan_heuristic_cost(pos1, pos2):
  dx = abs(pos1[0] - pos2[0])
  dy = abs(pos1[1] - pos2[1])
  return (dx + dy)

# diagonal distance 
def chebyshev_heuristic_cost(pos1, pos2):
  dx = abs(pos1[0] - pos2[0])
  dy = abs(pos1[1] - pos2[1])
  return (dx + dy) - min(dx, dy)

# octile distance
def octile_heuristic_cost(pos1, pos2):
  dx = abs(pos1[0] - pos2[0])
  dy = abs(pos1[1] - pos2[1])
  sqrt2 = 1.41421356
  return min(dx, dy)*sqrt2 + max(dx, dy) - min(dx, dy)

