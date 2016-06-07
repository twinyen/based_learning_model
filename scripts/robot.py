#!/usr/bin/env python

#robot.py implementation goes here
import rospy
import numpy as np
from read_config import read_config
from astar import astar_path
from mdp import mdp_policy
from model_based import model_based
from std_msgs.msg import Bool
from cse_190_assi_3.msg import AStarPath, PolicyList

class robot():
  def __init__(self):
    self.config = read_config()
    self.move_list = self.config['move_list']
    
    #self.move_list = self.config['move_list_8_direction']
    self.map_size = self.config['map_size']
    self.start = self.config['start']
    self.goal = self.config['goal']
    self.walls = self.config['walls']
    self.pits = self.config['pits']
    self.grid = self.create_grid()
  
    rospy.init_node("robot")
    
    #publish
    self.astar_publisher = rospy.Publisher("/results/path_list", AStarPath, queue_size = 100)  
    self.policy_publisher = rospy.Publisher("/results/policy_list", PolicyList, queue_size = 100)  
    self.simu_publisher = rospy.Publisher("/map_node/sim_complete", Bool, queue_size = 100)

  def create_grid(self):
    grid = np.ones(self.map_size)
    for p in self.pits:
      grid[p[0]][p[1]] = 999
    for w in self.walls:
      grid[w[0]][w[1]] = 888
	  
    grid[self.start[0]][self.start[1]] = 0
    grid[self.goal[0]][self.goal[1]] = 0
    return grid

if __name__ == '__main__':
  rb = robot()
  rospy.sleep(2)

  model = model_based(rb.grid, rb.start, rb.goal, rb.walls, rb.pits)
  # convert to probability
  for x in model:
    summ = sum(x)
    for y in range(0, len(x)):
      if(summ != 0):
        x[y] = x[y]*1.0/summ
      

  #print "model: \n", model, "\n\n"

  mypath = astar_path(rb.grid, rb.start, rb.goal, rb.walls, rb.pits, rb.move_list)
  policy = mdp_policy(0, model,rb.grid, rb.start, rb.goal, rb.walls, rb.pits, rb.move_list)
  policy1 = mdp_policy(1, model,rb.grid, rb.start, rb.goal, rb.walls, rb.pits, rb.move_list)
 
  correct = 0
  for i in range (0, len(policy)):
    for j in range (0, len(policy[0])):
      if(policy[i][j] == policy1[i][j]):
        correct += 1

  percentage = correct * 100.0 / (len(policy)*len(policy[0]))
  print "Optimal policy from MDP: \n", policy, "\n\n"
  print "Optimal policy from model based learning: \n", policy1, "\n\n"
  print "Percentage of correctness: \n", percentage, "%", "\n\n"
  #publish path and policy

  for x in mypath:
    path = AStarPath()
    path.data = x
    rospy.sleep(1)
    rb.astar_publisher.publish(path)  

  newpo = []
  for x in policy:
    for y in x:
      newpo.append(y)


  po = PolicyList()
  po.data = newpo
  rospy.sleep(1)
  rb.policy_publisher.publish(po)
  
  rospy.sleep(4)
  # publish finish simu
  rb.simu_publisher.publish(True)
  rospy.sleep(4)
  # shutdown
  rospy.signal_shutdown("Done")









