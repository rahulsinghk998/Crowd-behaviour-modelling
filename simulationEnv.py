"""
File for Pre-processing of collected data
"""
import math
import pickle
import numpy as np
import matplotlib.pyplot as plt


##>>Need a path smoothening function
##>>NEED A MODIFICATION

def storeSimulationEnvironment(datafile=0, agent_num = 1, plot = False):
    rect_bound = ((0, 0), 800, 800)
    rect=[((0,0),     200, 125),
          ((300,0),   200, 125),
          ((0,250),   50,  400),
          ((150,250), 100, 400),
          ((350,250), 100, 400),
          ((550,250), 100, 400),
          ((750,250), 50,  400),
          ((0,750),   200, 50),
          ((300,750), 200, 50),
          ((600,750), 200, 50),]    #[(startPoint), length_x, width_y]
    circle = []
    line   = []
    human  = []
    sim_layout = {'outline': rect_bound, 'rect':rect, 'circle':circle, 'line': line, 'human': human }
    return sim_layout


# Initialization of the meuseum environment    
def museumSimulationEnvironment(datafile=0, agent_num = 1, plot = False):
    ## Simulation Environment Boundary:: [(startPoint),length_x,width_y]
    rect_bound = ((0, 0), 800, 800)
    
    ## Static Obstacles in the Simulation environment
    # [(startPoint),length_x,width_y]-[(5/40 * 800, 800 - 6/40 * 800), 30/40*800, 6/40*800]
    rect   = [((100, 800-120), 600, 120),]
    
    # (centre, radius)
    circle = [((221.60, 526.56), 15.3),
              ((344.45, 522.35), 15.3),
              ((346.31, 363.97), 15.3),
              ((589.29, 507.34), 15.3),
              ((515.76, 477.39), 15.3),
              ((486.73, 368.44), 15.3),]
    
    # Linear Objects: ((x1,x2), (y1,y2), thickness)
    line   = [((221.6,  526.56), (344.45, 522.35), 5),
              ((344.45, 522.35), (346.31, 363.97), 5),
              ((589.29, 507.34), (515.76, 477.39), 5),
              ((515.76, 477.39), (486.73, 368.44), 5),
              ((208.86, 98.07),  (230.92, 100.07), 5),
              ((230.92, 100.07), (230.92, 0),      5),
              ((605.46, 96.55),  (582.37, 94.54),  5),  
              ((582.37, 94.54),  (582.37, 0),      5),]
    
    # Initialize the stored human path in the agent path
    # Add dynamic obstacles movement are known in prior 
    human_data             = getActorTrajectory(plot)
    human_path, agent_path = agentTimeSynchronize(human_agent_path = human_data, agent_num = agent_num)
    
    human_path_travel      = [[] for i in range(len(human_path))]
    human_path_remain      = human_path
    agent_path_remain      = agent_path
    agent_path_travel      = []
    
    # Humans Obstacles: [(human_num, frame_num, (pos_x, pos_y), (velocity_x,velocity_y), (acc_x, acc_y), (width -, length |))]
    humanpath = [human_path_travel, human_path_remain] # both have different structure--> check agentSimulation.py file  
    agentpath = [agent_path_travel, agent_path_remain] # both have different structure--> check agentSimulation.py file
    # Store current position used for plotting the human current position
    humanpos  = []
    agentpos  = []
        
    sim_layout = {'outline'  : rect_bound, 
                  'rect'     : rect, 
                  'circle'   : circle, 
                  'line'     : line, 
                  'humanpos' : humanpos, 
                  'humanpath': humanpath,
                  'agentpos' : agentpos, 
                  'agentpath': agentpath }
    
    return sim_layout


"""
Function: Read the annotated trajectory of human walking recording  and returns the processed path
Input: ?? - as of now nothing - the function itself reads the file from hard drive
Return: List of all agents trajectory. Each time instant is represented as (agent number {1,2,3,..}, frame number, (x,y) coordinate)
"""
def getActorTrajectory(plot = True):
    #load the stored trajectory
    with open ('/Users/rahulsingh/Desktop/Crowd-behaviour-modelling/dataset/agentpath_part1_278frames', 'rb') as fp:
        agentpath = pickle.load(fp)
        
    # load the perspective transformation matrix
    with open ('/Users/rahulsingh/Desktop/Crowd-behaviour-modelling/dataset/perspectiveTFmat_31july', 'rb') as fp:
        tfmat = pickle.load(fp)
        
    actor_num = 8
    new_path  = [[] for i in range(actor_num)]
    if plot:
        plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
        
    for i in range(actor_num):
        tf_midpoint = []
        new = old   = []
        for a, b, c in agentpath[i]:
            new = a
            if new == old:   #correct for any resets during manual marking
                tf_midpoint.pop()
                new_path[i].pop()
            xy    = (x,y) = ((c[1][0] + c[0][0])/2, (c[1][1] + c[0][1])/2)
            tf_xy = (-1*sum(tfmat[0]*(x,y,1))/sum(tfmat[2]*(x,y,1)) + 800, #flip and shift
                     -1*sum(tfmat[1]*(x,y,1))/sum(tfmat[2]*(x,y,1)) + 800)
            tf_midpoint.append(tf_xy)
            new_path[i].append((i+1, a ,tf_xy))
            old   = new
        
        new_path[i].reverse()
        xx = np.array(tf_midpoint).T
        if plot:
            plt.text(xx[0][0], xx[1][0], str(i+1), fontsize=18)
            plt.plot(xx[0], xx[1])
            
    if plot:
        plt.xlim((0, 800))
        plt.ylim((0, 800))
        plt.show()
    return new_path


"""
Function: In the simulation environment there is 1 agent while other are all real-human trajectories, so this function gives the trajectory of other humans which lie within the time frame of the simulated agent (human)
Input Param: All Human trajectory, human number replacing agent
Return: Human Trajectory points, Agent Trajectory points
"""
def agentTimeSynchronize(human_agent_path, agent_num, min_traj_points = 2):
    if agent_num<1 or agent_num>len(human_agent_path):
        print("Agent replacing human is out of index, Check!!")
        return None
    
    agent_path = human_agent_path[agent_num - 1]
    if len(agent_path)<min_traj_points:
        print("No tranjectory path for the human simulating agent")
        return None
    
    human_agent_path.remove(agent_path) # subtract the agent from total set
    start_frame = agent_path[-1][1]
    stop_frame  = agent_path[0][1]
    print("Agent Starting Frame: ", start_frame)
    print("Agent Stop Frame Num: ", stop_frame)
    
    # List of list to dynamically create and store real human-agent trajectory
    new_human_path = [[]]
    # iterate over all the agents and store the frames lying in between simulating agent/human
    for human in human_agent_path:
        for human_num, frame_num, path_xy in human:
            if frame_num >= start_frame and frame_num <= stop_frame:
                new_human_path[-1].append((human_num, frame_num, path_xy))
        if len(new_human_path[-1]):
            new_human_path.append([])
        
    new_human_path.pop()
    human_path = human_agent_path
    return new_human_path, agent_path