# mdp implementation needs to go here
import math
import numpy as np
from read_config import read_config
 
def mdp_policy(model_type, model_based, grid, start, goal, walls, pits, move_list):
  # init the value iter map
  val_iter = init_value_iter(grid, start, goal, walls, pits)
  # init policy
  policy = init_policy(grid, start, goal, walls, pits)
 

  # special states
  sp_s = []
  sp_s.append(goal)
  for x in pits:
    sp_s.append(x)
  for x in walls:
    sp_s.append(x)
 
  # do value interation
  update_val(model_type, model_based, grid, policy, move_list, sp_s, val_iter, walls) 
  return policy

def update_val(model_type, model_based, grid, policy, move_list, sp_s, val_iter, walls):
  #update each state except the special states
  config = read_config()
  max_iter = config["max_iterations"]
  threshold = config["threshold_difference"]
  
  for iteration in range(0,max_iter): 
    diff = 0
    for x in range(0, len(grid)):
      for y in range(0, len(grid[0])):
        if([x, y] not in sp_s):
          neighbors = find_neighbor([x,y], move_list, grid)
          #use function  
          maxfuc = mdp_fuc(model_type, model_based, grid, [x,y], neighbors, val_iter, walls, policy) 
          #update value 
          old_val = val_iter[x][y]
          val_iter[x][y]=maxfuc
          diff += abs(val_iter[x][y] - old_val)
    if diff <= threshold:
      break


def mdp_fuc(model_type, model_based, grid, pos, neighbors, val_iter, walls, policy):
  # get prob
  config = read_config()
  
  prob_f = config["prob_move_forward"]
  prob_b = config["prob_move_backward"]
  prob_l = config["prob_move_left"]
  prob_r = config["prob_move_right"]

  if(model_type == 1):
    prob_f =  model_based[pos[0]*len(grid[0]) + pos[1]][0]
    prob_b =  model_based[pos[0]*len(grid[0]) + pos[1]][1]
    prob_l =  model_based[pos[0]*len(grid[0]) + pos[1]][2]
    prob_r =  model_based[pos[0]*len(grid[0]) + pos[1]][3]

    #print "prob: \n", prob_f, prob_b, prob_l, prob_r
   

  config = read_config()
  goal_v = config["reward_for_reaching_goal"]
  pit_v = config["reward_for_falling_in_pit"]
  wall_v = config["reward_for_hitting_wall"]
  step_v = config["reward_for_each_step"]

  ga = config["discount_factor"]


   # neighbors
  R = neighbors[0] 
  L = neighbors[1] 
  D = neighbors[2] 
  U = neighbors[3] 

  # mywalls includes boundary
  mywalls = []
  for x in walls:
    mywalls.append(x)
    
  # get the future rewards
  if U[0] >= 0 and U[0] < len(grid) and U[1] >= 0 and U[1] < len(grid[0]):
    fu_u = val_iter[U[0]][U[1]]
  else:
    # its boundary
    fu_u = val_iter[pos[0]][pos[1]]
    mywalls.append(U)

  if D[0] >= 0 and D[0] < len(grid) and D[1] >= 0 and D[1] < len(grid[0]):
    fu_d = val_iter[D[0]][D[1]]
  else:
    # its boundary
    fu_d = val_iter[pos[0]][pos[1]]
    mywalls.append(D)

  if L[0] >= 0 and L[0] < len(grid) and L[1] >= 0 and L[1] < len(grid[0]):
    fu_l = val_iter[L[0]][L[1]]
  else:
    # its boundary
    fu_l = val_iter[pos[0]][pos[1]]
    mywalls.append(L)

  if R[0] >= 0 and R[0] < len(grid) and R[1] >= 0 and R[1] < len(grid[0]):
    fu_r = val_iter[R[0]][R[1]]
  else:
    # its boundary
    fu_r = val_iter[pos[0]][pos[1]]
    mywalls.append(R)

  fu_cur = val_iter[pos[0]][pos[1]]
    

  # contains all actions
  val_lists = []




  #Up
  if U in mywalls:
    upp = prob_f * ( wall_v + ga * fu_cur)
  else:
    upp = prob_f * ( step_v + ga * fu_u )
  
  if D in mywalls:
    dd = prob_b * ( wall_v + ga * fu_cur)
  else:
    dd = prob_b * ( step_v + ga * fu_d)
  
  if L in mywalls:
    ll = prob_l * ( wall_v + ga * fu_cur)
  else:
    ll = prob_l * ( step_v + ga * fu_l )
  
  if R in mywalls:
    rr = prob_r * ( wall_v + ga * fu_cur)
  else:
    rr = prob_r * ( step_v + ga * fu_r )
  
  total_up = upp + dd + ll + rr
  val_lists.append(total_up)
  #Down
  if U  in mywalls:
    uppd = prob_b * (  wall_v  + ga * fu_cur)
  else:
    uppd = prob_b * ( step_v + ga * fu_u )
  
  if D in mywalls:
    ddd = prob_f * (  wall_v  + ga * fu_cur)
  else:
    ddd = prob_f * ( step_v + ga * fu_d)
  
  if L  in mywalls:
    lld = prob_r * (  wall_v  + ga * fu_cur)
  else:
    lld = prob_r * ( step_v + ga * fu_l )
  
  if R  in mywalls:
    rrd = prob_l * (  wall_v  + ga * fu_cur)
  else:
    rrd = prob_l * ( step_v + ga * fu_r )
  
  total_d = uppd + ddd + lld + rrd
  val_lists.append(total_d)
  
  

  #Left
  if U  in mywalls:
    uppl= prob_r * (  wall_v  + ga * fu_cur)
  else:
    uppl= prob_r * ( step_v + ga * fu_u )
  
  if D  in mywalls:
    ddl= prob_l * (  wall_v  + ga * fu_cur)
  else:
    ddl= prob_l * ( step_v + ga * fu_d)
  
  if L in mywalls:
    lll= prob_f * (  wall_v + ga * fu_cur)
  else:
    lll= prob_f * ( step_v + ga * fu_l )
  
  if R  in mywalls:
    rrl= prob_b * (  wall_v  + ga * fu_cur)
  else:
    rrl= prob_b * ( step_v + ga * fu_r )
  
  total_l = uppl + ddl + lll + rrl
  val_lists.append(total_l)


  #Right
  if U in mywalls:
    uppr = prob_l * (  wall_v  + ga * fu_cur)
  else:
    uppr = prob_l * ( step_v + ga * fu_u )
  
  if D in mywalls:
    ddr = prob_r * (  wall_v  + ga * fu_cur)
  else:
    ddr = prob_r * ( step_v + ga * fu_d)
  
  if L  in mywalls:
    llr = prob_b * (  wall_v  + ga * fu_cur)
  else:
    llr = prob_b * ( step_v + ga * fu_l )
  
  if R  in mywalls:
    rrr = prob_f * (  wall_v  + ga * fu_cur)
  else:
    rrr = prob_f * ( step_v + ga * fu_r )
  
  total_r = uppr + ddr + llr + rrr
  val_lists.append(total_r)
  #print "total_up is : ", total_up

  #print "total_d is : ", total_d
  #print "total_l is : ", total_l
  #print "total_r is : ", total_r



  #print "val_lists:", val_lists
  #index of max
  max_index = val_lists.index(max(val_lists))
  #print "max_index :", max_index
  if max_index == 0:
    policy[pos[0]][pos[1]]= 'N'
  elif max_index == 1:
    policy[pos[0]][pos[1]]= 'S'
  elif max_index == 2:
    policy[pos[0]][pos[1]]= 'W'
  else:
    policy[pos[0]][pos[1]]= 'E'
 
  return max(val_lists)



def find_neighbor(pos, move_list, grid):
  neighbor_list = []
 
  for m in move_list:
    neigh_x = pos[0] + m[0]
    neigh_y = pos[1] + m[1]
    temp1 = [neigh_x, neigh_y]
    neighbor_list.append(temp1) 
  return neighbor_list
     
          



def init_policy(grid, start, goal, walls, pits):
  # init plicy
  policy = [[" " for x in range(len(grid[0]))] for y in range(len(grid))]
  # update special states
  policy[goal[0]][goal[1]] = "GOAL"
  for x in pits:
    policy[x[0]][x[1]] = "PIT"
  for x in walls:
    policy[x[0]][x[1]] = "WALL"
  return policy

  

def init_value_iter(grid, start, goal, walls, pits):
  config = read_config()
  goal_v = config["reward_for_reaching_goal"]
  pit_v = config["reward_for_falling_in_pit"]
  wall_v = config["reward_for_hitting_wall"]
  step_v = config["reward_for_each_step"]

  # init everything to zero
  V_iter = [[0 for x in range(len(grid[0]))] for y in range(len(grid))]
  #update value at special location
  V_iter[goal[0]][goal[1]] = goal_v
  for x in pits:
    V_iter[x[0]][x[1]] = pit_v
  for x in walls:
    V_iter[x[0]][x[1]] = wall_v
 
  return V_iter  




