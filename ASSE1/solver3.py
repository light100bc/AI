#!/usr/bin/python
import copy
import math
import sys
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


class State:
    def __init__(self, map, parent, action, pathCost):
        """
        Build a State Node
        :param map: LaserTank
        """
        self.map = map
        self.parent = parent
        self.action = action
        self.pathCost = pathCost
        self.heuriCost=heuriFuncSum(self)
        self.evaluateCost = self.pathCost+self.heuriCost

    def __lt__(self, other):
        return self.evaluateCost < other.evaluateCost

    def __eq__(self, item):
        """
        if A is in B return TRUE,
        'A is in B' means same tank position(even different directions) AND map hasn't changed(eg. block destroyed)
        :param self, a set of State class B represent visited States
        :param item, a State class A
        """
        # for i in range(self.count()):
        map1 = self.map
        map2 = item.map
        #is the position of user equal
        if (map1.player_x!=map2.player_x or map1.player_y!=map2.player_y): return False
        #is map equal
        for i in range(map1.y_size-2): # we dont compare the most outter fence
            row_count = i + 1
            for j in range(map1.x_size-2): # we dont compare the most outter fence
                col_count=j + 1
                symbol1 = map1.grid_data[row_count][col_count]
                symbol2 = map2.grid_data[row_count][col_count]
                # compare 2 map, except the cell of thank in the item.map (map2)
                # if all other cell are same, we can conclude the map is the same as well as the tank position(tank direction no need to included)
                if (symbol1 != symbol2):
                    return False
        return True

    def isGoal(self):
        return self.map.is_finished()

    def get_next_actions(self):
        """
        possible actions in the current state
        1.directly facing wall, CANT 'f'/'s'
        2.one side is wall, CANT 'r'/'l' according to the side
        3.return an empty list [], if no possble action
        *** add an action 'b'='r'+'r' ONLY for the initial step or when map is CHANGED
        :return: a set of possible actions
        """
        if (self.parent is None or self.map.grid_data!=self.parent.map.grid_data):
            next_actions=['s','f','r','l','b']
        else:
            next_actions = [ 's','f', 'r', 'l']
        map=self.map
        # get blocks at front/left/right relative to the heading direction
        if (map.player_heading == map.DOWN):
            front_block=map.grid_data[map.player_y+1][map.player_x]
            right_block=map.grid_data[map.player_y][map.player_x-1]
            left_block=map.grid_data[map.player_y][map.player_x+1]
        elif(map.player_heading == map.UP):
            front_block=map.grid_data[map.player_y-1][map.player_x]
            right_block=map.grid_data[map.player_y][map.player_x+1]
            left_block=map.grid_data[map.player_y][map.player_x-1]
        elif(map.player_heading == map.RIGHT):
            front_block=map.grid_data[map.player_y][map.player_x+1]
            right_block=map.grid_data[map.player_y+1][map.player_x]
            left_block=map.grid_data[map.player_y-1][map.player_x]
        elif(map.player_heading == map.LEFT):
            front_block=map.grid_data[map.player_y][map.player_x-1]
            right_block=map.grid_data[map.player_y-1][map.player_x]
            left_block=map.grid_data[map.player_y+1][map.player_x]

        #   choose actions
        if (front_block==LaserTankMap.OBSTACLE_SYMBOL):
            next_actions.remove('f')
            next_actions.remove('s')
        if (right_block==LaserTankMap.OBSTACLE_SYMBOL):
            next_actions.remove('r')
        if (left_block==LaserTankMap.OBSTACLE_SYMBOL):
            next_actions.remove('l')
        return next_actions

    def get_next_children(self):
        """
        a set of possible States with possible actions given the current State 'self'
        :func get_child(),get_next_actions()
        :param self, current State
        :return: a set of States
        """
        next_children=[]
        next_actions=self.get_next_actions()
        for action in next_actions:
            next_children.append(self.get_child(action))
        return next_children

    def get_child(self, action):
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
            child.heuriCost=heuriFuncSum(child)
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

def mapStatistic(state2):
    """
    static all the import feature of !!state2!!
    flag_pos [0]=x=col,[1]=y=row
    :param state:
    :return: a dictionary of {
            anti-tank count*2,anti-tank destroy,
            bridge count*2,moving bridge position*2(the 2nd can be a land),bridge become land,bridge moved
            2 teleport positions,flag position,player position*2}
    """
    if (state2.parent is None):
        state1=state2
        state0=state2
    else:
        state1=state2.parent
        state0=get_initial(state2)

    map0=state0.map
    map1=state1.map
    map2=state2.map

    anti_count_m0=0;anti_count_m1=0; anti_count_m2=0; anti_destroy=False;anti_destroy_total=0
    flag_pos_m2=[]
    player_pos_m0=[];player_pos_m1=[]; player_pos_m2=[]
    bridge_pos_m1=[]; bridge_pos_m2=[];bridge_to_land=False;bridge_moved=False #possible become a land
    bridge_count_m0=0;bridge_count_m1=0;bridge_count_m2=0;bridge_to_land_total=0
    tele_pos1_m2=[];tele_pos2_m2=[];has_tele=False
    tele_count=0

    # !! row=y,col=x
    player_pos_m1.append(map1.player_x);   player_pos_m1.append(map1.player_y)
    player_pos_m2.append(map2.player_x);   player_pos_m2.append(map2.player_y)
    for row_count in range(map1.y_size-2):
        row_count=row_count+1
        for col_count in range(map1.x_size-2):
            col_count=col_count+1
            symbol_m0 = map0.grid_data[row_count][col_count]
            symbol_m1=map1.grid_data[row_count][col_count]
            symbol_m2=map2.grid_data[row_count][col_count]

            if(is_Anti_tank(symbol_m0)):anti_count_m0=anti_count_m0+1
            if(is_Anti_tank(symbol_m1)):anti_count_m1=anti_count_m1+1
            if(is_Anti_tank(symbol_m2)):anti_count_m2=anti_count_m2+1

            if (symbol_m2==LaserTankMap.FLAG_SYMBOL):
                flag_pos_m2.append(col_count); flag_pos_m2.append(row_count)

            if (symbol_m2==LaserTankMap.TELEPORT_SYMBOL):
                tele_count=tele_count+1
                has_tele=True
                if (tele_count==1):tele_pos1_m2.append(col_count);tele_pos1_m2.append(row_count)
                if (tele_count==2):tele_pos2_m2.append(col_count);tele_pos2_m2.append(row_count)

            if (symbol_m1==LaserTankMap.BRIDGE_SYMBOL and symbol_m2!=LaserTankMap.BRIDGE_SYMBOL): #means this bridge is moved
                bridge_pos_m1 = [col_count,row_count];bridge_moved=True
                index= is_bridge_moved(map1.grid_data[row_count-1][col_count-1],map2.grid_data[row_count-1][col_count-1])
                if(index!=0):  bridge_pos_m2=[col_count+1,row_count+1]
                if (index==2): bridge_to_land=True
                index=is_bridge_moved(map1.grid_data[row_count + 1][col_count ],map2.grid_data[row_count + 1][col_count])
                if (index != 0):  bridge_pos_m2 = [col_count, row_count+1]
                if (index == 2): bridge_to_land = True
                index=is_bridge_moved(map1.grid_data[row_count][col_count+1],map2.grid_data[row_count][col_count+1])
                if (index != 0):  bridge_pos_m2=[col_count+1,row_count]
                if (index == 2): bridge_to_land = True
                index=is_bridge_moved(map1.grid_data[row_count][col_count-1],map2.grid_data[row_count][col_count-1])
                if (index != 0):  bridge_pos_m2=[col_count-1,row_count]
                if (index == 2): bridge_to_land = True

    if (anti_count_m2!=anti_count_m1):anti_destroy=True
    anti_destroy_total=anti_count_m0-anti_count_m2

    res= {}
    res["anti_tank_destory"]=anti_destroy;res["player_pos_m2"]=player_pos_m2
    res["flag_pos_m2"]=flag_pos_m2;res["tele_pos1_m2"]=tele_pos1_m2;res["tele_pos2_m2"]=tele_pos2_m2;res["has_tele"]=has_tele
    res["moved_bridge_pos_m2"]=bridge_pos_m2;res["bridge_to_land"]=bridge_to_land;res["bridge_moved"]=bridge_moved
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
        h_tele1=abs(player_pos[0]-tele_pos1[0])+abs(player_pos[1]-tele_pos1[1])+abs(flag_pos[0]-tele_pos1[0])+abs(flag_pos[0]-tele_pos1[1])
        h_tele2=abs(player_pos[0]-tele_pos2[0])+abs(player_pos[1]-tele_pos2[1])+abs(flag_pos[0]-tele_pos2[0])+abs(flag_pos[0]-tele_pos2[1])
        return min(h_normal,h_tele1,h_tele2)


def heuriFuncBridge(state1,state2):
    """
    if bridge push near to the water is GOOD,away from the water is BAD,not change distance is MODERATE
    if bridge fall into the water is VERY GOOD
    if bridge fall into the water and closer to the F or T is VERY GOOD
    """

def heuriFuncFire(is_shoted,index):
    """
    if kill an anti-tank VERY GOOD
    let's assume if no anti-tank, after 1/2 of the map be visited will reach F
    so if no reach yet means anti-tank
    so return 1/2 of the map size
    :param state1:
    :param state2:
    :return:
    """
    if(is_shoted): #count anti-tank
        return -index
    return 0

def heuriFuncSum(state2):
    """
    heuristic from state1 to state2 by action
    :param map current map
    :return: a h distance and a distance included into the Pathcost
    """

    state2_static=mapStatistic(state2) #contain all the ele needded in all kinds of heuristics(avoid loops)
    #manhatton with/without tele
    h=heuriFuncManhattan2(state2_static.get("player_pos_m2"),state2_static.get("flag_pos_m2"),
                            state2_static.get("tele_pos1_m2"),state2_static.get("tele_pos2_m2"))\
        #+heuriFuncOnlyF(action)
    return h

def heuristicPoints(state1,action,state2):
    """
    we assume these points might be a necessary when going to the goal.
    reset pathCost=C the search will start from here (choose C according to the assumption)
    1. anti-tank destroyed
    2. bridge into river
    :param state1:
    :param action:
    :param state2:
    :return:
    """
    state2_static = mapStatistic(state2)
    p=0
    if(state2_static.get("anti_tank_destory")):
        p=(state2.map.x_size-2)/2
    if(state2_static.get("bridge_to_land")):
        p=(state2.map.x_size-2)/2
    return -p

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

    input_file = "testcases/t2_puzzle_maze.txt"
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
    initState = State(game_map, None, None, 0)
    frontier = []
    visited = []

    frontier.append(initState)
    heapq.heapify(frontier)

    visited_count = 0
    frontier_count=0
    action_count=0

    while frontier:

        node = heapq.heappop(frontier)
        if node.action: print("choose node from frontier: " , getPath(node),
                              "h: ",node.heuriCost,"p: ",node.pathCost,"e: ",node.evaluateCost)
        node.map.render()

        if node.isGoal():
            print("We Reached the goal, Jolly Good!")
            break;

        next_children=node.get_next_children() #all the successors
        for child in next_children:
            action_count=action_count+1
            if child and child not in visited:
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
        if (node not in visited):visited.append(node)
        visited_count = visited_count + 1

    actions=getPath(node)
    print(actions)
    print("total node visited: ", visited_count)
    print("total node frontier: ", frontier_count)
    print("total action: ", action_count)
    print ("steps: ",len(actions))

    # Write the solution to the output file
    write_output_file(output_file, actions)


if __name__ == '__main__':
    # main(sys.argv[1:])
    main()
