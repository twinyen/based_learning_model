# astar implementation needs to go here
import math
import numpy as np

def astar_path(grid, start, goal, walls, pits, move_list):
  
  print "grid is: "
  print grid
  #print "pit is: ", pits
  #print "wall is: ", walls
  
  #moving pattern [x,y] + [a,b]
  #if x+a > row or < 0 -> out of bound
  #same for y
  #if [x+a,y+b] = pit or wall -> not a valid move (block)
  #if [x+a,y+b] = goal -> finish
  #if [x+a,y+b] in explore[] -> skip  
  explored = []
  frontier = []
  paths = []
  heu = []
  paths.append([start])
  frontier.append(start)
  #explored.append(start)

  while (len(frontier) > 0):
    #print "Len of frontier", len(frontier)
    #print frontier
    temp_fron = frontier[:]
    print "temp_fron:", temp_fron
    heu = []
    for f in temp_fron:
      if f != goal:
      #  frontier.remove(f)
        for m in move_list:
          nm = [f[0]+m[0], f[1]+m[1]]
          if nm[0] >= grid.shape[0] or nm[0] < 0:
            continue
          if nm[1] >= grid.shape[1] or nm[1] < 0:
            continue
          # check if next move hit the walls or go to pits
          if not is_valid_move(nm, walls, pits, explored) and nm != goal:
            continue
          print nm , "<<"
          #raw_input()
          if nm not in frontier:
            heu_cost = heuristic_cost(nm,goal)
            heu.append(heu_cost)
            print "heu ", heu


            frontier.append(nm)
          #print "==========", frontier

          #heuristic of next move to goal
          #heu_cost = heuristic_cost(nm,goal)
          #heu.append(heu_cost)
          #print "heu: ", heu
          #validate if frontier is the next of path
          temps = paths[:]
          for p in temps:
            if is_neighbour(p[-1], f):
              temp_path = p[:]

              temp_path.append(f)
              paths.append(temp_path)

            #paths.remove(p)
      else:
        temps = paths[:]
        for p in temps:
          if is_neighbour(p[-1], f):
            temp_path = p[:]
            temp_path.append(f)
            paths.append(temp_path)

      explored.append(f)
      frontier.remove(f)

  retPaths = []
  for p in paths:
    if p[-1] == goal:
      retPaths.append(p)


  print "I found ", len(retPaths) , " paths"
  print retPaths
  '''
  for path in frontier:
    temp_node = path[-1]

    for move in move_list:
      next_move = [temp_node[0]+move[0], temp_node[1]+move[1]]
      #print "next_move" , next_move
      if next_move[0] >= grid.shape[0] or next_move[0] < 0:
        continue
      if next_move[1] >= grid.shape[1] or next_move[1] < 0:
        continue
      #check if next move hit the walls or go to pits
      if is_valid_move(next_move,walls,pits,explored):
        continue

      new_path = path[:]
      new_path.append(next_move)
      explored.append(next_move)

      frontier.append(new_path)

    #frontier.remove(path)
    for f in frontier:
      if f[-1] == goal:
        print f
   '''
def is_valid_move(move, walls, pits, explored):
  if move in walls or move in pits or move in explored:
    return False
  return True

def is_neighbour(pos1, pos2):
  move_list = [[0,1], [0,-1], [1,0], [-1,0]]
  for m in move_list:
    if (pos1[0] + m[0] == pos2[0]) and (pos1[1] + m[1] == pos2[1]):
      return True
  return False

def heuristic_cost(pos1, pos2):
  return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1] - pos2[1])**2)



