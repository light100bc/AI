#!/usr/bin/python
import queue
import sys
import time

from laser_tank import LaserTankMap

"""
Template file for you to implement your solution to Assignment 1.

COMP3702 2020 Assignment 1 Support Code
"""
"""
2 3
"""

#
#
# Code for any classes or functions you need can go here.
#
#

class State:
    def __init__(self,map,action,player_x,player_y,heading,parent,pathCost):
        self.map=map
        self.action = action
        self.player_x = player_x
        self.player_y = player_y
        self.heading = heading
        self.parent=parent
        self.heuri_persist=0
        self.pathCost=pathCost
        self.totalCost=0 # pathCost+heuriFuncSum(self,initial_static)


    def __lt__(self, other):
        return self.totalCost < other.totalCost

    def __eq__(self, item):
        if (self.player_x!=item.player_x or self.player_y!=item.player_y): return False
        if (self.heading!=item.heading):return False
        return self.map == item.map

# def heuriFuncSum(state):

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

def is_Anti_tank(symbol):
    """
    :param symbol:
    :return: whether the symbol represent an alive anti-tank(up,right,left,down)
    """
    if (symbol == LaserTankMap.ANTI_TANK_DOWN_SYMBOL) or \
            (symbol == LaserTankMap.ANTI_TANK_UP_SYMBOL) or \
            (symbol == LaserTankMap.ANTI_TANK_LEFT_SYMBOL) or \
            (symbol == LaserTankMap.ANTI_TANK_RIGHT_SYMBOL):
        return True
    return False

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

        anti_count = 0
        flag_pos = []
        bridge_count = 0;
        bridge_pos = [];
        bridge_heuri = 0  # bridge move near flag/tele is GOOD
        tele_pos1 = [];
        tele_pos2 = [];
        tele_count = 0

        # !! row=y,col=x
        for i in range(len(map) - 2):
            row_count = i + 1
            for j in range(len(map[0]) - 2):
                col_count = j + 1
                symbol_m2 = map[row_count][col_count]
                if (is_Anti_tank(symbol_m2)): anti_count = anti_count + 1
                if (symbol_m2 == LaserTankMap.FLAG_SYMBOL):
                    flag_pos.append(col_count);
                    flag_pos.append(row_count)
                if (symbol_m2 == LaserTankMap.TELEPORT_SYMBOL):
                    tele_count = tele_count + 1
                    if (tele_count == 1): tele_pos1.append(col_count);tele_pos1.append(row_count)
                    if (tele_count == 2): tele_pos2.append(col_count);tele_pos2.append(row_count)
                if (symbol_m2 == LaserTankMap.BRIDGE_SYMBOL):
                    bridge_count = bridge_count + 1
                    bridge_pos.append([col_count, row_count])

        # use existing Manhatton func, brilliant!, bridge_pos replace player_pos
        # finally add all bridge Manhatton together
        i = 0
        for i in range(bridge_count):
            bridge_heuri = bridge_heuri + heuriFuncManhattan2(bridge_pos[i], flag_pos, tele_pos1, tele_pos2)

        res = {}
        res["anti_count"] = anti_count
        res["flag_pos"] = flag_pos
        res["tele_pos1"] = tele_pos1;
        res["tele_pos2"] = tele_pos2
        res["bridge_count"] = bridge_count;
        res["bridge_heuri"] = bridge_heuri
        return res

def heuriFuncSum(state2,initial_static,rev_heuri_rcd=None):
    """
    heuristic from state1 to state2 by action
    !!! this function is excuted only if the action is succeed.
    So if the action is SHOT and the map contain water/anti, we will use different heuri Func
    !!! rev_heuri_rcd is reverse tank caculated heuristic
    IF EXIST use it, IF NOT use manhatton + max(rev_heuri_rcd)
    :param map current map
    :return: a h distance and a distance included into the Pathcost
    """

    if (initial_static["anti_count"]!=0 or initial_static["bridge_count"]!=0):
        if (state2.action == 's' or state2.action==None):
            state2_static = mapStatistic(state2.map)  # contain all the ele needded in all kinds of heuristics(avoid loops)
            state2.heuri_persist= heuriFuncFire(initial_static["anti_count"],state2_static["anti_count"]) \
                                      + 0.3*state2_static["bridge_heuri"]\
                                      + 2*heuriFuncBridge(initial_static["bridge_count"],state2_static["bridge_count"])\
                                      - 5



    h_manhattan = heuriFuncManhattan2([state2.player_x, state2.player_y], initial_static["flag_pos"],
                                      initial_static["tele_pos1"], initial_static["tele_pos2"])
    # if(rev_heuri_rcd):
    #     key = (state2.player_x, state2.player_y)
    #     if(key in rev_heuri_rcd):
    #         h_manhattan=rev_heuri_rcd[key]
    #         state2.heuri_persist=state2.heuri_persist
    #     else:
    #         h_manhattan = max(h_manhattan,max(rev_heuri_rcd.values()))#make sure sum is one
    #         state2.heuri_persist = state2.heuri_persist
    h= h_manhattan*1.5 + state2.heuri_persist
    return 0


def getPath(node):
    actions=[]
    while node.parent:
        actions.append(node.action)
        node = node.parent
    # reverse it to get the correct order
    actions.reverse()
    return actions

def get_action(action):
    actionset=['s','f','l','r']
    if (action=='l'): actionset.remove('r');actionset.remove('l')
    if (action == 'r'): actionset.remove('l')
    return actionset


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

def main(arglist):
# def main():
    input_file = arglist[0]
    output_file = arglist[1]
    #
    # input_file = "testcases/t1_bridgeport.txt"
    # output_file = "res.txt"
    start_time=time.time()
    # Read the input testcase file
    game_map = LaserTankMap.process_input_file(input_file)

    actions = []
    #actionset=['s','f','l','r']

    #
    #
    # Code for your main method can go here.
    #
    # Your code should find a sequence of actions for the agent to follow to reach the goal, and store this sequence
    # in 'actions'.
    #
    #tuple=>explored, fortier<=env,action


    """
    ================================= main code ===============================
    """

    temp=[doc[:] for doc in game_map.grid_data]
    initial_static = mapStatistic(temp)
    start=State(temp,None,game_map.player_x,game_map.player_y,game_map.player_heading,None,0)
    start.totalCost = start.pathCost + heuriFuncSum(start, initial_static)
    fringe = queue.PriorityQueue()
    fringe.put(start)
    visited_frontier={}
    temp=tuple(tuple(x) for x in start.map)
    visited_frontier[(temp,start.player_x,start.player_y,start.heading)] = start.pathCost

    frontier_count=1
    action_count=1
    visited_count=0



    while not fringe.empty():
        frontier_count = frontier_count-1
        current = fringe.get()
        visited_count = visited_count +1
        # if current.action: print("choose node from frontier: " , getPath(current),
        #                        "p: ",current.pathCost)
        temp = [doc[:] for doc in current.map]
        game_map.grid_data=temp
        game_map.player_x=current.player_x
        game_map.player_y = current.player_y
        game_map.player_heading=current.heading
        # game_map.render()
        if game_map.is_finished():
            # print("reached the end, jolly good")
            #print(explored[current.id])
            break

        actionset=get_action(current.action)
        for action in actionset:  # "simulate" executing actions

            action_count = action_count + 1

            temp = [doc[:] for doc in current.map]
            game_map.grid_data = temp
            game_map.player_x = current.player_x
            game_map.player_y = current.player_y
            game_map.player_heading = current.heading
            index=game_map.apply_move(action)
            cost_so_far = current.pathCost + 1
            neighbor = State(game_map.grid_data, action, game_map.player_x, game_map.player_y, game_map.player_heading,
                             current, cost_so_far)
            neighbor.totalCost = neighbor.pathCost + heuriFuncSum(neighbor, initial_static)
            temp = tuple(tuple(x) for x in neighbor.map)
            if (index==0 and ((temp, neighbor.player_x, neighbor.player_y, neighbor.heading) not in visited_frontier or visited_frontier[(temp, neighbor.player_x, neighbor.player_y, neighbor.heading)]>neighbor.totalCost)):
                frontier_count = frontier_count + 1
                visited_frontier[(temp, neighbor.player_x, neighbor.player_y, neighbor.heading)] = neighbor.pathCost
                fringe.put(neighbor)


    actions = getPath(current)
    print(arglist[0])
    print("v: ",visited_count," a: ", action_count," f:", frontier_count)
    print(actions,len(actions))
    print(time.time()-start_time)
    # Write the solution to the output file
    write_output_file(output_file, actions)


if __name__ == '__main__':
    # main(sys.argv[1:])
    testset=["testcases/t1_bridgeport.txt","testcases/t1_bridgeport.txt","testcases/t1_bridgeport.txt","testcases/t1_bridgeport.txt","testcases/t1_bridgeport.txt","testcases/t1_bridgeport.txt","testcases/t1_bridgeport.txt","testcases/t1_bridgeport.txt",]
    for i in testset:
        main([i,"res.txt"])

