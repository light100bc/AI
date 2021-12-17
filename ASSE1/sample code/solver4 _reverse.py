#!/usr/bin/python
import copy
import math
import sys
import time

from laser_tank import LaserTankMap

"""
Template file for you to implement your solution to Assignment 1.

COMP3702 2020 Assignment 1 Support Code
"""

#
#
# Code for any classes or functions you need can go here.
#
#
import heapq

class visited_dir_State:
    """
    visited directed state record
    """
    def __init__(self, map, pathCost):
        """
        Build a State Node
        :param map: LaserTank
        """
        self.map = map
        self.pathCost = pathCost
        # self.heuriCost_persist=0
        # self.heuriCost=heuriFuncSum(self)
        # self.evaluateCost = self.pathCost+self.heuriCost
         #fire/bridge after anti/bridge,next C step should continue

    def __lt__(self, other):
        return self.evaluateCost < other.evaluateCost

    def __eq__(self, item):
        """
        if A is in B return TRUE,
        'A is in B'
        same direction, same grid, same tank coord
        :param self, a set of State class B represent visited States
        :param item, a State class A
        """
        # for i in range(self.count()):
        map1 = self.map
        map2 = item.map
        #is the position of user equal
        if (map1.player_x!=map2.player_x or map1.player_y!=map2.player_y): return False
        if (self.map.player_heading!=item.map.player_heading):return False
        return map1.grid_data == map2.grid_data


class State:
    def __init__(self, map, parent, action, pathCost,intial_state):
        """
        Build a State Node
        :param map: LaserTank
        """
        self.map = map
        self.parent = parent
        self.action = action
        self.initial=intial_state
        self.pathCost = pathCost
        self.heuriCost_persist=0
        self.heuriCost=heuriFuncSum(self,None) #initial should include heuri-bridge and anti
        self.evaluateCost = self.pathCost+self.heuriCost

    def __lt__(self, other):
        return self.evaluateCost < other.evaluateCost

    def __eq__(self, item):
        """
        if A is in B return TRUE,
        'A is in B' means same tank position(even different directions) AND map hasn't changed(eg. block destroyed)
        assumption: everyting shold be done before leave a cell, can not come back to the same cell if the map has not changed
        :param self, a set of State class B represent visited States
        :param item, a State class A
        """
        # for i in range(self.count()):
        map1 = self.map
        map2 = item.map
        #is the position of user equal
        if (map1.player_x!=map2.player_x or map1.player_y!=map2.player_y): return False
        if (self.heuriCost != item.heuriCost or self.heuriCost_persist!=item.heuriCost_persist): return False
        return map1.grid_data == map2.grid_data

    def isGoal(self):
        return self.map.is_finished()

    def get_next_actions(self):
        """
        A. (NOT IN USE ILLEGAL?)possible actions in the current state
            1.directly facing wall, CANT 'f'/'s'
            2.one side is wall, CANT 'r'/'l' according to the side
            3.return an empty list [], if no possble action
        B. add an action 'b'='r'+'r' ONLY for
            1. the initial step (bridge/articulation of the graph)
            2. when map is CHANGED
            3. after teleport (use coord to find whether teleport, ice will also trigger 'b'
                                assumption: a map has little ice=>less 'b', not too much impact
                                            a map with lots of ice=>less land node,not too much impact)
        :return: a set of possible actions
        """
        if (self.parent is None or self.map.grid_data!=self.parent.map.grid_data):
                #or abs(self.map.player_x-self.parent.map.player_x)+abs(self.map.player_x-self.parent.map.player_x)>1):
            next_actions=['s','f','r','l','b']
        else:
            next_actions = [ 's','f', 'r', 'l']
        map=self.map
        # get blocks at front/left/right relative to the heading direction
        # if (map.player_heading == map.DOWN):
        #     front_block=map.grid_data[map.player_y+1][map.player_x]
        #     right_block=map.grid_data[map.player_y][map.player_x-1]
        #     left_block=map.grid_data[map.player_y][map.player_x+1]
        # elif(map.player_heading == map.UP):
        #     front_block=map.grid_data[map.player_y-1][map.player_x]
        #     right_block=map.grid_data[map.player_y][map.player_x+1]
        #     left_block=map.grid_data[map.player_y][map.player_x-1]
        # elif(map.player_heading == map.RIGHT):
        #     front_block=map.grid_data[map.player_y][map.player_x+1]
        #     right_block=map.grid_data[map.player_y+1][map.player_x]
        #     left_block=map.grid_data[map.player_y-1][map.player_x]
        # elif(map.player_heading == map.LEFT):
        #     front_block=map.grid_data[map.player_y][map.player_x-1]
        #     right_block=map.grid_data[map.player_y-1][map.player_x]
        #     left_block=map.grid_data[map.player_y+1][map.player_x]
        #
        # #   choose actions
        # if (front_block==LaserTankMap.OBSTACLE_SYMBOL):
        #     next_actions.remove('f')
        #     next_actions.remove('s')
        # if (right_block==LaserTankMap.OBSTACLE_SYMBOL):
        #     next_actions.remove('r')
        # if (left_block==LaserTankMap.OBSTACLE_SYMBOL):
        #     next_actions.remove('l')
        return next_actions

    def get_next_children(self,rev_heuri_rcd):
        """
        a set of possible States with possible actions given the current State 'self'
        :func get_child(),get_next_actions()
        :param self, current State
        :return: a set of States
        """
        next_children=[]
        next_actions=self.get_next_actions()
        for action in next_actions:
            next_children.append(self.get_child(action,rev_heuri_rcd))
        return next_children

    def get_child(self, action,rev_heuri_rcd):
        """
        Create a child State of the State node according to the action
        including the transition function and cost function
        ! 'b'='r'+'r'
        :func heuriFuncSum()
        :param action:
        :return a child State, *if no child, return 0
        """
        child = copy.deepcopy(self)
        if (action=='b'):
            index = child.map.apply_move('r')
            index = child.map.apply_move('r')
            child.action = ['r','r']
            child.pathCost = self.pathCost + 2
        else:
            index = child.map.apply_move(action)
            child.action = action
            child.pathCost = self.pathCost + 1
        # if is a dead end
        if index != 0:
            return 0
        else:
            child.parent = self
            child.heuriCost=heuriFuncSum(child,rev_heuri_rcd)
            child.evaluateCost=child.pathCost + child.heuriCost
            return child

"""===========================================subFuncs for heuriFuncs========================"""
def is_Anti_tank(symbol):
    """
    :param symbol:
    :return: whether the symbol represent an alive anti-tank(up,right,left,down)
    """
    if (symbol==LaserTankMap.ANTI_TANK_DOWN_SYMBOL) or \
            (symbol == LaserTankMap.ANTI_TANK_UP_SYMBOL) or \
            (symbol == LaserTankMap.ANTI_TANK_LEFT_SYMBOL) or \
            (symbol == LaserTankMap.ANTI_TANK_RIGHT_SYMBOL):
        return True
    return False

def is_bridge_moved(symbol1,symbol2):
    """
    if (symbol1!=bridge and symbol2==bridge) or (symbol1==water and symbol2==land)
    :param symbol1:
    :param symbol2:
    :return: 0 not moved, 1 moved, 2 moved and become land
    """
    index=0
    if (symbol1!=LaserTankMap.BRIDGE_SYMBOL and symbol2==LaserTankMap.BRIDGE_SYMBOL):
        index=1
    if (symbol1==LaserTankMap.WATER_SYMBOL and symbol2==LaserTankMap.LAND_SYMBOL):
        index=2
    return index

def mapStatistic(map):
    """
    static all the import feature of !!state2!!
    flag_pos [0]=x=col,[1]=y=row
    :param state:
    :return: a dictionary of {
            anti-tank count*2,anti-tank destroy,
            bridge count*2,moving bridge position*2(the 2nd can be a land),bridge become land,bridge moved,bridge heuristic!! all B length to tele and flag
            2 teleport positions,flag position,player position*2}
    """

    map2=map
    anti_count=0
    flag_pos=[]
    bridge_count=0;bridge_pos=[];bridge_heuri=0 #bridge move near flag/tele is GOOD
    tele_pos1=[];tele_pos2=[];tele_count=0

    # !! row=y,col=x
    for i in range(map2.y_size-2):
        row_count=i+1
        for j in range(map2.x_size-2):
            col_count=j+1
            symbol_m2=map2.grid_data[row_count][col_count]
            if(is_Anti_tank(symbol_m2)):anti_count=anti_count+1
            if (symbol_m2==LaserTankMap.FLAG_SYMBOL):
                flag_pos.append(col_count); flag_pos.append(row_count)
            if (symbol_m2==LaserTankMap.TELEPORT_SYMBOL):
                tele_count=tele_count+1
                if (tele_count==1):tele_pos1.append(col_count);tele_pos1.append(row_count)
                if (tele_count==2):tele_pos2.append(col_count);tele_pos2.append(row_count)
            if (symbol_m2==LaserTankMap.BRIDGE_SYMBOL):
                bridge_count=bridge_count+1
                bridge_pos.append([col_count,row_count])

    # use existing Manhatton func, brilliant!, bridge_pos replace player_pos
    # finally add all bridge Manhatton together
    i=0
    for i in range(bridge_count):
        bridge_heuri = bridge_heuri + heuriFuncManhattan2(bridge_pos[i],flag_pos,tele_pos1,tele_pos2)

    res= {}
    res["anti_count"]=anti_count
    res["flag_pos"]=flag_pos
    res["tele_pos1"]=tele_pos1;res["tele_pos2"]=tele_pos2
    res["bridge_count"]=bridge_count;res["bridge_heuri"]=bridge_heuri
    return res

def get_initial(node):
    """
    get the initial Node of the given Node(State)
    :param state:
    :return:
    """
    while node.parent:
        node = node.parent
    return node

"""=============================================heuriFuncs===================================="""

def heuriFuncOnlyF(action):
    """
    if not 'f', dont count cost
    :param action:
    :return:
    """
    if (action != 'f'): return -1
    else: return 0

def heuriFuncManhattan2(player_pos,flag_pos,tele_pos1=None,tele_pos2=None):
    """
    if tele_pos=None, distance= Manhattan without tele
    if has tele_pos, distance=min(Manhattan with tele,Manhattan without tele)
    :param player_pos:
    :param flag_pos:
    :param tele_pos:
    :return: Manhattan distance (with/without tele)
    """
    h_normal = abs(player_pos[0] - flag_pos[0]) + abs(player_pos[1] - flag_pos[1])
    if (tele_pos1==[]):
        return h_normal
    else:
        h_tele1=abs(player_pos[0]-tele_pos1[0])+abs(player_pos[1]-tele_pos1[1])+abs(tele_pos2[0]-flag_pos[0])+abs(tele_pos2[1]-flag_pos[1])
        h_tele2=abs(player_pos[0]-tele_pos2[0])+abs(player_pos[1]-tele_pos2[1])+abs(tele_pos1[0]-flag_pos[0])+abs(tele_pos1[1]-flag_pos[1])
        return min(h_normal,h_tele1,h_tele2)


def heuriFuncBridge(initial_bridge,current_bridge):
    """
    if bridge fall into the water is GOOD
    """
    if(initial_bridge>current_bridge):
        return -(initial_bridge-current_bridge)
    return 0

def heuriFuncFire(initial_anti,current_anti):
    """
    more kill, closer to the goal
    :param state1:
    :param state2:
    :return:
    """
    if(initial_anti>current_anti):
        return -(initial_anti-current_anti)
    return 0

def heuriFuncSum(state2,rev_heuri_rcd):
    """
    heuristic from state1 to state2 by action
    !!! this function is excuted only if the action is succeed.
    So if the action is SHOT and the map contain water/anti, we will use different heuri Func
    !!! rev_heuri_rcd is reverse tank caculated heuristic
    IF EXIST use it, IF NOT use manhatton + max(rev_heuri_rcd)
    :param map current map
    :return: a h distance and a distance included into the Pathcost
    """


    if (state2.initial["anti_count"]!=0 or state2.initial["bridge_count"]!=0):
        if (state2.action == 's' or state2.action==None):
            state2_static = mapStatistic(state2.map)  # contain all the ele needded in all kinds of heuristics(avoid loops)
            state2.heuriCost_persist= heuriFuncFire(state2.initial["anti_count"],state2_static["anti_count"]) \
                                      + state2_static["bridge_heuri"]/2\
                                      + heuriFuncBridge(state2.initial["bridge_count"],state2_static["bridge_count"])
    key=(state2.map.player_x,state2.map.player_y)

    h_manhattan = heuriFuncManhattan2([state2.map.player_x, state2.map.player_y], state2.initial["flag_pos"],
                                      state2.initial["tele_pos1"], state2.initial["tele_pos2"])
    if(rev_heuri_rcd):
        if(key in rev_heuri_rcd):
            h_manhattan=rev_heuri_rcd[key]
            state2.heuriCost_persist=state2.heuriCost_persist
        else:
            h_manhattan = max(h_manhattan,max(rev_heuri_rcd.values()))#make sure sum is one
            state2.heuriCost_persist = state2.heuriCost_persist
    h= h_manhattan + state2.heuriCost_persist
    return h

"""==============================other functions==================================="""


def getPath(node):
    actions=[]
    while node.parent:
        if (node.action==['r','r']): actions.append('r');actions.append('r')
        else:   actions.append(node.action)
        node = node.parent
    # reverse it to get the correct order
    actions.reverse()
    return actions

def write_output_file(filename, actions):
    """
    Write a list of actions to an output file. You should use this method to write your output file.
    :param filename: name of output file
    :param actions: list of actions where is action is in LaserTankMap.MOVES
    """
    f = open(filename, 'w')
    for i in range(len(actions)):
        f.write(str(actions[i]))
        if i < len(actions) - 1:
            f.write(',')
    f.write('\n')
    f.close()


# def main(arglist):
def main():
    # input_file = arglist[0]
    # output_file = arglist[1]
    start=time.time()

    input_file = "testcases/t2_brickyard.txt"
    output_file = "res.txt"

    # Read the input testcase file
    game_map = LaserTankMap.process_input_file(input_file)

    actions = []

    #
    #
    # Code for your main method can go here.
    #
    # Your code should find a sequence of actions for the agent to follow to reach the goal, and store this sequence
    # in 'actions'.
    #
    #

    """
    ================= preprocess get heuristic distance =======================
    reverse visit (flag->initial)
    ice = end point
    water = +2
    block = +2
    Tele = min(tele1,tele2)
    anti=block
    """
    rev_game_map=copy.deepcopy(game_map)
    y=rev_game_map.player_y ;x=rev_game_map.player_x
    for i in range(rev_game_map.y_size-2):
        row_count=i+1
        for j in range(rev_game_map.x_size-2):
            col_count=j+1
            symbol_m2 = rev_game_map.grid_data[row_count][col_count]
            if (is_Anti_tank(symbol_m2)): rev_game_map.grid_data[row_count][col_count]=LaserTankMap.OBSTACLE_SYMBOL
            if (symbol_m2==LaserTankMap.WATER_SYMBOL): rev_game_map.grid_data[row_count][col_count]=LaserTankMap.LAND_SYMBOL
            if (symbol_m2==LaserTankMap.BRIDGE_SYMBOL): rev_game_map.grid_data[row_count][col_count]=LaserTankMap.LAND_SYMBOL
            if (symbol_m2 == LaserTankMap.BRICK_SYMBOL): rev_game_map.grid_data[row_count][col_count] = LaserTankMap.LAND_SYMBOL
            if (symbol_m2 == LaserTankMap.MIRROR_DL_SYMBOL): rev_game_map.grid_data[row_count][col_count] = LaserTankMap.LAND_SYMBOL
            if (symbol_m2 == LaserTankMap.MIRROR_DR_SYMBOL): rev_game_map.grid_data[row_count][col_count] = LaserTankMap.LAND_SYMBOL
            if (symbol_m2 == LaserTankMap.MIRROR_UL_SYMBOL): rev_game_map.grid_data[row_count][col_count] = LaserTankMap.LAND_SYMBOL
            if (symbol_m2 == LaserTankMap.MIRROR_UR_SYMBOL): rev_game_map.grid_data[row_count][col_count] = LaserTankMap.LAND_SYMBOL
            if (symbol_m2 == LaserTankMap.FLAG_SYMBOL): rev_game_map.player_y=row_count;rev_game_map.player_x=col_count; rev_game_map.grid_data[row_count][col_count] = LaserTankMap.LAND_SYMBOL

    rev_game_map.grid_data[y][x] = LaserTankMap.FLAG_SYMBOL

    """
    ========================= main process ==================================================
    """
    stat=mapStatistic(game_map) #initial statics of the map
    initState = State(game_map, None, None, 0,stat)

    frontier = []
    visited = []#no direction
    visited2= []#has direction
    visited_frontier={}

    frontier.append(initState)
    heapq.heapify(frontier)
    node_copy = visited_dir_State(initState.map, initState.pathCost)
    visited2.append(node_copy)
    visited_frontier[initState.map]=initState.pathCost

    rev_stat = mapStatistic(rev_game_map)  # initial statics of the map
    rev_initState = State(rev_game_map, None, None, 0, rev_stat)
    rev_frontier=[]
    rev_visited=[]
    rev_visited2=[]
    rev_frontier.append(rev_initState)
    heapq.heapify(rev_frontier)
    rev_node_copy = visited_dir_State(rev_initState.map, rev_initState.pathCost)
    rev_visited2.append(rev_node_copy)
    rev_heuri_rcd={}

    visited_count = 0
    frontier_count=0
    action_count=0

    flag=False

    while frontier:

        node = heapq.heappop(frontier)

        if node.action: print("choose node from frontier: " , getPath(node),
                              "h: ",node.heuriCost,"p: ",node.pathCost,"e: ",node.evaluateCost)
        node.map.render()

        if node.isGoal():
            print("We Reached the goal, Jolly Good!")
            break;


        if (frontier_count>1000 and rev_frontier and flag==False): #if large map use reverse
            rev_node = heapq.heappop(rev_frontier)
            rev_node.map.render()
            if rev_node.isGoal():
                print("We Reached the goal, Jolly Good!")
                flag=True
            rev_next_children = rev_node.get_next_children(None)  # all the successors
            rev_heuri_rcd[(rev_node.map.player_x,rev_node.map.player_y)]=rev_node.pathCost
            for rev_child in rev_next_children:
                if rev_child and rev_child not in rev_visited:
                    rev_node_copy = visited_dir_State(rev_child.map, rev_child.pathCost)
                    if (rev_node_copy not in rev_visited2):  # a more strict visited condition
                        visited2.append(rev_node_copy)
                        heapq.heappush(rev_frontier, rev_child)
            rev_visited.append(rev_node)

        next_children=node.get_next_children(rev_heuri_rcd) #all the successors
        for child in next_children:
            action_count=action_count+1
            if child and (child not in visited or (child.map in visited_frontier and child.pathCost<visited_frontier[child.map])):
                node_copy = visited_dir_State(child.map, child.pathCost)
                if (node_copy not in visited2 or (child.map in visited_frontier and child.pathCost<visited_frontier[child.map])):   #a more strict visited condition
                    visited_frontier[node_copy.map]=node_copy.pathCost
                    visited2.append(node_copy)
                    heapq.heappush(frontier, child)
                    frontier_count = frontier_count + 1
                    print("its children: ", getPath(child))
                    child.map.render()
        # 1.visited put at last, with __eq__
        # because we not allow come back to the same cell without changing the map
        # in other words, all the actions should be done before leaving a cell, unless the map is changed later on
        # on one cell only turn l/r ONCE is allowed (if the map is not changed)
        # 2. use "if" to exam whether node already in visited
        # because we put into visited at last, there can be two map with the same tank position(different direction)
        # we dont need it
        visited.append(node)
        visited_count = visited_count + 1




    actions=getPath(node)
    print(actions)
    print("total node visited: ", visited_count)
    print("total node frontier: ", frontier_count)
    print("total action: ", action_count)
    print ("steps: ",len(actions))
    end=time.time()
    print("time: ", end-start)

    # Write the solution to the output file
    write_output_file(output_file, actions)


if __name__ == '__main__':
    # main(sys.argv[1:])
    main()
