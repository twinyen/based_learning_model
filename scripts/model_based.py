from read_config import read_config
import random as rd

def model_based(grid, start, goal, walls, pits):
 
  # model grid, initialize prob to zeros
  model_grid = []
  for x in range (0, len(grid)):
    for y in range(0, len(grid[0])):
      model_grid.append([0,0,0,0])
 
  for x in range (0, 100):
    newres = get_transition_prob(grid,  model_grid, start, goal, walls, pits) 
    model_grid = newres

  
  # return policy
  return model_grid


def get_transition_prob(grid, model_grid, start, goal, walls, pits):
  # run robot on a fixed policy
  config = read_config()
  policy = config["policy"]
  prob_f = config["prob_move_forward"]
  prob_b = config["prob_move_backward"]
  prob_l = config["prob_move_left"]
  prob_r = config["prob_move_right"]


  #print "start: \n", start
  curr = [0,0]
  curr[0] = start[0]
  curr[1] = start[1]
  # if robot does not read goal or pit
  #print "##################new iteration\n"
  while(curr != goal and curr not in pits):
    # make a move
    # get a policy
    poli = policy[ curr[0]*len(grid[0]) + curr[1]]   
    #print "policy: \n", poli

    if poli == "N":
      move_f = [-1, 0]
      move_b = [1, 0]
      move_l = [0, -1]
      move_r = [0, 1]

    if poli == "S":
      move_f = [1, 0]
      move_b = [-1, 0]
      move_l = [0, 1]
      move_r = [0, -1]


    if poli == "W":
      move_f = [0, -1]
      move_b = [0, 1]
      move_l = [1, 0]
      move_r = [-1, 0]


    if poli == "E":
      move_f = [0, 1]
      move_b = [0, -1]
      move_l = [-1, 0]
      move_r = [1, 0]

    #  move the robot
    c = rd.uniform (0.0, 1.0)
    # get where robot move to
 
    f1 = prob_f
    b1 = prob_f + prob_b
    l1 = b1 + prob_l

    #print "Move \n"
    nextpo = [0, 0]

    # robot move forward
    if(c>= 0.0 and c <= f1):
      nextpo[0] = curr[0] + move_f[0]
      nextpo[1] = curr[1] + move_f[1]
      # if next pos is not in walls or not out of boundary -> move to next position
      #print "In forward \n"

      #print "currrrr nefore: \n", curr
      # increase prob_f
      if(poli == "N"):
        model_grid[ curr[0]*len(grid[0]) + curr[1]][0] += 1   
      if(poli == "S"):
        model_grid[ curr[0]*len(grid[0]) + curr[1]][0] += 1   
      if(poli == "W"):
        model_grid[ curr[0]*len(grid[0]) + curr[1]][0] += 1   
      if(poli == "E"):
        model_grid[ curr[0]*len(grid[0]) + curr[1]][0] += 1   

      if(nextpo not in walls and nextpo[0] >= 0 and nextpo[0] < len(grid) and nextpo[1] >= 0 and nextpo[1] < len(grid[0])):  
        curr[0] = nextpo[0]
        curr[1] = nextpo[1]
      #print "currrrr: \n", curr

     



    # robot move backward
    elif(c> f1 and c <= b1):
      nextpo[0] = curr[0] + move_b[0]
      nextpo[1] = curr[1] + move_b[1]
   
      # if next pos is not in walls or not out of boundary -> move to next position
     

      #print "In backward \n"
      #print "currrrr nefore: \n", curr
      # increase prob_f
      if(poli == "N"):
        model_grid[ curr[0]*len(grid[0]) + curr[1]][1] += 1   
      if(poli == "S"):
        model_grid[ curr[0]*len(grid[0]) + curr[1]][1] += 1   
      if(poli == "W"):
        model_grid[ curr[0]*len(grid[0]) + curr[1]][1] += 1   
      if(poli == "E"):
        model_grid[ curr[0]*len(grid[0]) + curr[1]][1] += 1   
       
      if(nextpo not in walls and nextpo[0] >= 0 and nextpo[0] < len(grid) and nextpo[1] >= 0 and nextpo[1] < len(grid[0])):  
        curr[0] = nextpo[0]
        curr[1] = nextpo[1]
      #print "currrrr: \n", curr


    # robot move left 
    elif(c> b1 and c <= l1):
      nextpo[0] = curr[0] + move_l[0]
      nextpo[1] = curr[1] + move_l[1]
      # if next pos is not in walls or not out of boundary -> move to next position
      #print "In left \n"

      #print "currrrr nefore: \n", curr
      # increase prob_f
      if(poli == "N"):
        model_grid[ curr[0]*len(grid[0]) + curr[1]][2] += 1   
      if(poli == "S"):
        model_grid[ curr[0]*len(grid[0]) + curr[1]][2] += 1   
      if(poli == "W"):
        model_grid[ curr[0]*len(grid[0]) + curr[1]][2] += 1   
      if(poli == "E"):
        model_grid[ curr[0]*len(grid[0]) + curr[1]][2] += 1  

      if(nextpo not in walls and nextpo[0] >= 0 and nextpo[0] < len(grid) and nextpo[1] >= 0 and nextpo[1] < len(grid[0])):  
        curr[0] = nextpo[0]
        curr[1] = nextpo[1]

      #print "currrrr: \n", curr


    # robot move right
    else:
      nextpo[0] = curr[0] + move_r[0]
      nextpo[1] = curr[1] + move_r[1]
      
      # if next pos is not in walls or not out of boundary -> move to next position
      
      #print "right\n"
      #print "currrrr nefore: \n", curr
      # increase prob_f
      if(poli == "N"):
        model_grid[ curr[0]*len(grid[0]) + curr[1]][3] += 1   
      if(poli == "S"):
        model_grid[ curr[0]*len(grid[0]) + curr[1]][3] += 1   
      if(poli == "W"):
        model_grid[ curr[0]*len(grid[0]) + curr[1]][3] += 1   
      if(poli == "E"):
        model_grid[ curr[0]*len(grid[0]) + curr[1]][3] += 1   

      if(nextpo not in walls and nextpo[0] >= 0 and nextpo[0] < len(grid) and nextpo[1] >= 0 and nextpo[1] < len(grid[0])):  
        curr[0] = nextpo[0]
        curr[1] = nextpo[1]

      #print "currrrr: \n", curr



    #print "model:   \n", model_grid
  return model_grid    
