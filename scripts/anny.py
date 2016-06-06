# astar implementation needs to go here
import math
import numpy as np

def astar_path(grid, start, goal, walls, pits, move_list):
  
  print "grid is: "
  print grid

  cmap = init_cost_map(grid, start, move_list)
  print "cmap : "
  print cmap

  hmap = init_heu_map(grid, goal, move_list)
  print "hmap : "
  print hmap
 
  sumV = sumA(cmap, hmap) 
  print "sumV", sumV
       

  test = set()
  test.add((start[0],start[1]))


  short_path = cal_astar(start, goal, sumV, move_list, grid)
  print"shor path: ", short_path       

def sumA(cmap, hmap):
  sumV = [[0 for x in range(len(cmap[0]))] for y in range(len(cmap))]
  for i in range(0, len(cmap)):
    for j in range(0, len(cmap[0])):
      sumV[i][j] = cmap[i][j] + hmap[i][j]
  return sumV



def cal_astar(start, goal, sumV, move_list, grid):
  op = set()
  clo = set()
  cur = start
  
  print"stat: ", start
  op.add(start)
  
  #while op is not empty
  while op:
    cur = minop(op, sumV)
    # if cur is goal, recieve all the path
    if(cur == goal):
      path = []
      while come_from[cur]:
        path.append(cur)
        cur = come_from[cur]
      #append last one
      path.append(cur)
      return path[::-1]

    op.remove(cur)
    clo.add(cur)
    
    #loop over the neighbors of current
    neighbors = find_neighbor(cur, move_list, grid, clo)
    for n in neighbors:
      if n in clo:
        continue
      if n in op:
        come_from[n] = cur
      else:
        come_from[n] = cur    
        op.add(n)

def minop (op, sumV):
  # make the first element as min value
  for x in op:
    minn = op(0)
    break;
  # find the real min value
  for x in op:
    temp = sumV[x[0]][x[1]]
    if(temp < minn):
      minn = temp

  return minn

def init_heu_map(grid,goal, move_list):
  heu_map = [[0 for x in range(len(grid[0]))] for y in range(len(grid))]

  row = len(heu_map)
  col = len(heu_map[0])

  for i in range (0,row):
    for j in range (0,col):
      heu_map[i][j] = 0

  queue2 = []
  queue2.append(goal)
  explored2 = []
  update_cost(queue2, explored2, -1, heu_map, move_list)
  return heu_map

def init_cost_map(grid, start, move_list):
  cost_map  = [[0 for x in range(len(grid[0]))] for y in range(len(grid))]

  row = len(cost_map)
  col = len(cost_map[0])


  # initialize cost map to zero
  for i in range(0, row):
    for j in range(0, col):
      cost_map[i][j] = 0



  # cost for start point
  start_x = start[0]
  start_y = start[1]
  cost_map[start_x][start_y] = 0

  # put the start point to queue
  queue = []
  queue.append(start)
  explored = []
  update_cost(queue, explored, -1, cost_map, move_list)

  return cost_map
 

def update_cost( q, exp, value, cost_map, move_list):
  #termination
  if len(q) == 0: return 
  # new queue for next generation
  qq = []
  while len(q) > 0:
    temp = q.pop()
    #check if temp in exp list
    if(in_exp(temp, exp) == False):
      #update cost at temp
      cost_map[temp[0]][temp[1]] = value + 1
      #get the valid neighbors
      neighbors = find_neighbor(temp, move_list, cost_map, exp)
      # put neighbor in queue
      for x in neighbors:
        qq.append(x)
      # put temp into exp
      exp.append(temp)

    
  update_cost(qq, exp, value + 1 , cost_map, move_list)


def find_neighbor(pos, move_list, grid, exp):
  neighbor_list = []
 
  for m in move_list:
    neigh_x = pos[0] + m[0]
    neigh_y = pos[1] + m[1]
    temp1 = [neigh_x, neigh_y]
    if neigh_x >= 0 and neigh_x < len(grid) and neigh_y >= 0 and neigh_y < len(grid[0]):
      if (in_exp(temp1, exp)== False):
        neighbor_list.append(temp1) 
  return neighbor_list
     

def in_exp(temp, exp):
  res = False  
  for x in exp:
    if x == temp:
      return True
  return res
           


 
def is_neighbour(pos1, pos2):
  move_list = [[0,1], [0,-1], [1,0], [-1,0]]
  for m in move_list:
    if (pos1[0] + m[0] == pos2[0]) and (pos1[1] + m[1] == pos2[1]):
      return True
  return False
