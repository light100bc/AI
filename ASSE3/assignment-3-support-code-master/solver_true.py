import math
import random
import time

from laser_tank import LaserTankMap, DotDict

"""
Template file for you to implement your solution to Assignment 3. You should implement your solution by filling in the
following method stubs:
    run_value_iteration()
    run_policy_iteration()
    get_offline_value()
    get_offline_policy()
    get_mcts_policy()
    
You may add to the __init__ method if required, and can add additional helper methods and classes if you wish.

To ensure your code is handled correctly by the autograder, you should avoid using any try-except blocks in your
implementation of the above methods (as this can interfere with our time-out handling).

COMP3702 2020 Assignment 3 Support Code
"""


class Solver:

    def __init__(self, game_map):
        self.game_map = game_map.make_clone()
        #
        # TODO
        # Write any environment preprocessing code you need here (e.g. storing teleport locations).
        #
        # You may also add any instance variables (e.g. root node of MCTS tree) here.
        #
        # The allowed time for this method is 1 second, so your Value Iteration or Policy Iteration implementation
        # should be in the methods below, not here.
        #
        for j in range(1, self.game_map.y_size - 1):
                for k in range(1, self.game_map.x_size - 1):
                    if self.game_map.grid_data[j][k]==LaserTankMap.FLAG_SYMBOL:
                        self.F_y=j
                        self.F_x=k
                    if (j == self.game_map.player_y and k == self.game_map.player_x):
                        self.Start_y = j
                        self.Start_x = k


        self.values = None
        self.policy = None
        self.prob = game_map.t_success_prob

    def run_value_iteration(self):
        """
        Build a value table and a policy table using value iteration, and store inside self.values and self.policy.
        """
        """
        trick: to lazy to wright transition function, just use in-build with prob=1
        """
        values = [[[0 for _ in LaserTankMap.DIRECTIONS]
                   for __ in range(1, self.game_map.y_size - 1)]
                  for ___ in range(1, self.game_map.x_size - 1)]
        policy = [[[-1 for _ in LaserTankMap.DIRECTIONS]
                   for __ in range(1, self.game_map.y_size - 1)]
                  for ___ in range(1, self.game_map.x_size - 1)]

        #
        # TODO
        # Write your Value Iteration implementation here.
        #
        # When this method is called, you are allowed up to [state.time_limit] seconds of compute time. You should stop
        # iterating either when max_delta < epsilon, or when the time limit is reached, whichever occurs first.
        #
        start = time.time()
        current = start
        epsilon = self.game_map.epsilon
        max_delta = math.inf
        #to speed up, initial value use a propotion to manhattan
        #Empircally find a good coeff
        #!!!Empircally initial must <0
        norm=(1/2)*abs(self.game_map.benchmark/(abs(self.Start_y-self.F_y)+abs(self.Start_x-self.F_x)))
        for j in range(1, self.game_map.y_size - 1):
            for k in range(1, self.game_map.x_size - 1):
                for i in LaserTankMap.DIRECTIONS:
                    if (self.game_map.grid_data[j][k] == LaserTankMap.LAND_SYMBOL
                            or self.game_map.grid_data[j][k] == LaserTankMap.TELEPORT_SYMBOL
                            or self.game_map.grid_data[j][k] == LaserTankMap.ICE_SYMBOL):
                        values[k-1][j-1][i]=(-abs(j-self.F_y)-abs(k-self.F_x))*norm


        # loop
        # F initial set 0
        # wall and water set 0, because already have collision_cost and game_over_cost
        # above cell do not change V value any more!!!
        count=1
        while (abs(max_delta) >= epsilon):# and current-start<=self.game_map.time_limit):
            count=count+1
            current = time.time()
            print("time: ",current - start,"max_delta: ",max_delta," count: ",count)
            max_delta = -math.inf
            for j in range(1, self.game_map.y_size - 1):
                for k in range(1, self.game_map.x_size - 1):
                    for i in LaserTankMap.DIRECTIONS:
                        if (self.game_map.grid_data[j][k] == LaserTankMap.LAND_SYMBOL
                                or self.game_map.grid_data[j][k] == LaserTankMap.TELEPORT_SYMBOL
                                or self.game_map.grid_data[j][k] == LaserTankMap.ICE_SYMBOL):
                            self.game_map.player_x = j
                            self.game_map.player_y = k
                            self.player_heading = i
                            current_value = values[k-1][j-1][i]
                            max_value = -math.inf  # choose the action result in the max V value
                            for a in LaserTankMap.MOVES:  # do an action, NO SHOT!
                                if (a == 's'): continue
                                elif (a == 'l'):
                                    self.game_map.player_x = k
                                    self.game_map.player_y = j
                                    self.game_map.player_heading = i
                                    # if (self.game_map.player_heading==LaserTankMap.UP): direction = LaserTankMap.LEFT
                                    # if (self.game_map.player_heading == LaserTankMap.LEFT): direction = LaserTankMap.DOWN
                                    # if (self.game_map.player_heading == LaserTankMap.DOWN): direction = LaserTankMap.RIGHT
                                    # if (self.game_map.player_heading == LaserTankMap.RIGHT): direction = LaserTankMap.UP
                                    estimate_value = self.game_map.apply_move('l') + self.game_map.gamma * \
                                                     values[self.game_map.player_x-1][
                                                         self.game_map.player_y-1][self.game_map.player_heading]
                                    if (max_value < estimate_value):
                                        max_policy = 'l'
                                        max_value = estimate_value
                                elif (a == 'r'):
                                    self.game_map.player_x = k
                                    self.game_map.player_y = j
                                    self.game_map.player_heading = i
                                    # if (self.game_map.player_heading == LaserTankMap.UP): direction = LaserTankMap.RIGHT
                                    # if (self.game_map.player_heading == LaserTankMap.LEFT): direction = LaserTankMap.UP
                                    # if (self.game_map.player_heading == LaserTankMap.DOWN): direction = LaserTankMap.LEFT
                                    # if (self.game_map.player_heading == LaserTankMap.RIGHT): direction = LaserTankMap.DOWN
                                    estimate_value = self.game_map.apply_move('r') + self.game_map.gamma * \
                                                     values[self.game_map.player_x-1][
                                                         self.game_map.player_y-1][self.game_map.player_heading]
                                    if (max_value < estimate_value):
                                        max_policy = 'r'
                                        max_value = estimate_value
                                else:  # move forward
                                    self.game_map.player_x = k
                                    self.game_map.player_y = j
                                    self.game_map.player_heading = i
                                    temp_map=self.game_map.make_clone() #because we want to CHEAT(prob=1) so must have a copy
                                    temp_map.t_success_prob=1 #CHEAT haha, use in build method
                                    estimate_value = 0
                                    #all the move done on estimate map
                                    #1st s' success
                                    instant_cost=temp_map.apply_move('f')
                                    if (instant_cost == self.game_map.collision_cost):
                                        estimate_value = estimate_value + self.game_map.t_success_prob * (
                                                    instant_cost + temp_map.gamma * \
                                                    values[self.game_map.player_x - 1][self.game_map.player_y - 1][
                                                        self.game_map.player_heading])
                                    elif (instant_cost == self.game_map.game_over_cost): #absorbing state
                                        estimate_value = estimate_value + self.game_map.t_success_prob * instant_cost
                                    else:
                                        estimate_value = estimate_value +  self.game_map.t_success_prob * (
                                                    instant_cost + temp_map.gamma * \
                                                    values[temp_map.player_x - 1][temp_map.player_y - 1][
                                                        temp_map.player_heading])
                                    #2nd s' not move
                                    estimate_value = estimate_value + (1-self.game_map.t_success_prob)/5*(temp_map.move_cost+temp_map.gamma * \
                                                            values[self.game_map.player_x-1][self.game_map.player_y-1][self.game_map.player_heading])
                                    #other 4 s'
                                    if(self.game_map.player_heading==LaserTankMap.UP):next_pos=[(k-1,j),(k-1,j+1),(k+1,j),(k+1,j+1)]
                                    if (self.game_map.player_heading == LaserTankMap.DOWN): next_pos = [(k - 1, j-1),(k - 1, j ),(k + 1, j), (k + 1, j-1)]
                                    if (self.game_map.player_heading == LaserTankMap.LEFT): next_pos = [(k+1, j+1),(k , j + 1),(k, j-1),(k +1, j-1)]
                                    if (self.game_map.player_heading == LaserTankMap.RIGHT): next_pos = [(k  -1, j-1),(k , j - 1),(k, j+1),(k -1, j+1)]
                                    for position in next_pos:
                                        temp_map.player_x=position[0]
                                        temp_map.player_y=position[1]
                                        temp_map.player_heading=self.game_map.player_heading
                                        #ICE,LAND,TELE,FLAG use V(s'), WATER use V(s)
                                        instant_cost=temp_map.apply_move('f')
                                        if (instant_cost==self.game_map.collision_cost):
                                            estimate_value=estimate_value + (1-self.game_map.t_success_prob)/5*(instant_cost+ temp_map.gamma * \
                                            values[self.game_map.player_x - 1][self.game_map.player_y - 1][self.game_map.player_heading])
                                        elif(instant_cost==self.game_map.game_over_cost ):
                                            estimate_value = estimate_value + (1-self.game_map.t_success_prob) / 5 * instant_cost
                                        else:
                                            estimate_value = estimate_value + (1-self.game_map.t_success_prob)/5*(instant_cost + temp_map.gamma * \
                                                values[temp_map.player_x - 1][temp_map.player_y - 1][temp_map.player_heading])
                                    if (max_value < estimate_value):
                                        max_policy = 'f'
                                        max_value = estimate_value
                            values[k-1][j-1][i] = max_value
                            policy[k-1][j-1][i] = max_policy
                            max_delta = max(max_delta, max_value - current_value)
            current = time.time()
        # store the computed values and policy
        self.values = values
        self.policy = policy
        #!!!!!!!!!!!!!!!!!! 不可以一直左转:两个地方，一个是5中position错误，一个是self.heading改为self.game_map.heading


    def run_policy_iteration(self):
        """
        Build a value table and a policy table using policy iteration, and store inside self.values and self.policy.
        """
        """
        The difference between Vi and Pi is 
        in Pi I initialize by policy, in each iter choose best policy and then renew the value
        in Vi I initialize by value, in each iter choose best value and then renew the policy
        code is very similar!!! but faster because the terminate condition is convergence of policy
        """
        values = [[[0 for _ in LaserTankMap.DIRECTIONS]
                   for __ in range(1, self.game_map.y_size - 1)]
                  for ___ in range(1, self.game_map.x_size - 1)]
        policy = [[[-1 for _ in LaserTankMap.DIRECTIONS]
                   for __ in range(1, self.game_map.y_size - 1)]
                  for ___ in range(1, self.game_map.x_size - 1)]

        #
        # TODO
        # Write your Policy Iteration implementation here.
        #
        # When this method is called, you are allowed up to [state.time_limit] seconds of compute time. You should stop
        # iterating either when max_delta < epsilon, or when the time limit is reached, whichever occurs first.
        #

        """basically it should be similar to the Vi
        only difference is the role of policy[] and value[] exchange (so does their place in the code)"""
        start = time.time()
        current = start
        # to speed up, initial policy use point to the flag
        # I should use manhatton as v-value and iter one loop to set initial policy
        # but to make clear that this is pi not vi, I set policy directly
        for j in range(1, self.game_map.y_size - 1):
            for k in range(1, self.game_map.x_size - 1):
                for i in LaserTankMap.DIRECTIONS:
                    if (self.game_map.grid_data[j][k] == LaserTankMap.LAND_SYMBOL
                            or self.game_map.grid_data[j][k] == LaserTankMap.TELEPORT_SYMBOL
                            or self.game_map.grid_data[j][k] == LaserTankMap.ICE_SYMBOL):
                        x_coord = k-self.F_x
                        y_coord = (-j) - (-self.F_y)
                        if(y_coord>x_coord and y_coord>=-x_coord): #above F
                            if(i==1):policy[k - 1][j - 1][i] ='f'
                            if(i==0):policy[k - 1][j - 1][i] ='r'
                            if (i == 2): policy[k - 1][j - 1][i] = 'l'
                            if (i == 3): policy[k - 1][j - 1][i] = 'r'
                        elif (y_coord <= x_coord and y_coord > -x_coord): # at right of F
                            if(i==1):policy[k - 1][j - 1][i] ='r'
                            if(i==0):policy[k - 1][j - 1][i] ='l'
                            if (i == 2): policy[k - 1][j - 1][i] = 'f'
                            if (i == 3): policy[k - 1][j - 1][i] = 'r'
                        elif (y_coord < x_coord and y_coord <= -x_coord): # below F
                            if(i==1):policy[k - 1][j - 1][i] ='r'
                            if(i==0):policy[k - 1][j - 1][i] ='f'
                            if (i == 2): policy[k - 1][j - 1][i] = 'r'
                            if (i == 3): policy[k - 1][j - 1][i] = 'l'
                        else: #(y_coord >= x_coord and y_coord < -x_coord): # at left of F
                            if(i==1):policy[k - 1][j - 1][i] ='l'
                            if(i==0):policy[k - 1][j - 1][i] ='r'
                            if (i == 2): policy[k - 1][j - 1][i] = 'r'
                            if (i == 3): policy[k - 1][j - 1][i] = 'f'
        # now get the initial V-value from initial policy
        for j in range(1, self.game_map.y_size - 1):
            for k in range(1, self.game_map.x_size - 1):
                for i in LaserTankMap.DIRECTIONS:
                    if (self.game_map.grid_data[j][k] == LaserTankMap.LAND_SYMBOL
                            or self.game_map.grid_data[j][k] == LaserTankMap.TELEPORT_SYMBOL
                            or self.game_map.grid_data[j][k] == LaserTankMap.ICE_SYMBOL):
                        self.game_map.player_x = j
                        self.game_map.player_y = k
                        self.player_heading = i
                        current_value = values[k - 1][j - 1][i]
                        max_value = -math.inf  # choose the action result in the max V value
                        a=policy[k - 1][j - 1][i]  # !!!!different from vi, already have an action, NO SHOT!
                        if (a == 's'):
                            continue
                        elif (a == 'l'):
                            self.game_map.player_x = k
                            self.game_map.player_y = j
                            self.game_map.player_heading = i
                            # if (self.game_map.player_heading==LaserTankMap.UP): direction = LaserTankMap.LEFT
                            # if (self.game_map.player_heading == LaserTankMap.LEFT): direction = LaserTankMap.DOWN
                            # if (self.game_map.player_heading == LaserTankMap.DOWN): direction = LaserTankMap.RIGHT
                            # if (self.game_map.player_heading == LaserTankMap.RIGHT): direction = LaserTankMap.UP
                            estimate_value = self.game_map.apply_move('l') + self.game_map.gamma * \
                                             values[self.game_map.player_x - 1][
                                                 self.game_map.player_y - 1][self.game_map.player_heading]
                            values[k - 1][j - 1][i] = estimate_value
                        elif (a == 'r'):
                            self.game_map.player_x = k
                            self.game_map.player_y = j
                            self.game_map.player_heading = i
                            # if (self.game_map.player_heading == LaserTankMap.UP): direction = LaserTankMap.RIGHT
                            # if (self.game_map.player_heading == LaserTankMap.LEFT): direction = LaserTankMap.UP
                            # if (self.game_map.player_heading == LaserTankMap.DOWN): direction = LaserTankMap.LEFT
                            # if (self.game_map.player_heading == LaserTankMap.RIGHT): direction = LaserTankMap.DOWN
                            estimate_value = self.game_map.apply_move('r') + self.game_map.gamma * \
                                             values[self.game_map.player_x - 1][
                                                 self.game_map.player_y - 1][self.game_map.player_heading]
                            values[k - 1][j - 1][i] = estimate_value
                        else:  # move forward
                            self.game_map.player_x = k
                            self.game_map.player_y = j
                            self.game_map.player_heading = i
                            temp_map = self.game_map.make_clone()  # because we want to CHEAT(prob=1) so must have a copy
                            temp_map.t_success_prob = 1  # CHEAT haha, use in build method
                            estimate_value = 0
                            # all the move done on estimate map
                            # 1st s' success
                            instant_cost = temp_map.apply_move('f')
                            if (instant_cost == self.game_map.collision_cost):
                                estimate_value = estimate_value + self.game_map.t_success_prob * (
                                        instant_cost + temp_map.gamma * \
                                        values[self.game_map.player_x - 1][self.game_map.player_y - 1][
                                            self.game_map.player_heading])
                            elif (instant_cost == self.game_map.game_over_cost):  # absorbing state
                                estimate_value = estimate_value + self.game_map.t_success_prob * instant_cost
                            else:
                                estimate_value = estimate_value + self.game_map.t_success_prob * (
                                        instant_cost + temp_map.gamma * \
                                        values[temp_map.player_x - 1][temp_map.player_y - 1][
                                            temp_map.player_heading])
                            # 2nd s' not move
                            estimate_value = estimate_value + (1 - self.game_map.t_success_prob) / 5 * (
                                    temp_map.move_cost + temp_map.gamma * \
                                    values[self.game_map.player_x - 1][self.game_map.player_y - 1][
                                        self.game_map.player_heading])
                            # other 4 s'
                            if (self.game_map.player_heading == LaserTankMap.UP): next_pos = [(k - 1, j + 1), (
                                k - 1, j - 1 + 1), (k + 1, j - 1 + 1), (k + 1, j + 1)]
                            if (self.game_map.player_heading == LaserTankMap.DOWN): next_pos = [(k - 1, j - 1),
                                                                                                (k - 1,
                                                                                                 j + 1 - 1), (
                                                                                                    k + 1,
                                                                                                    j + 1 - 1),
                                                                                                (k + 1, j - 1)]
                            if (self.game_map.player_heading == LaserTankMap.LEFT): next_pos = [(k + 1, j + 1),
                                                                                                (k - 1 + 1,
                                                                                                 j + 1), (
                                                                                                    k - 1 + 1,
                                                                                                    j - 1),
                                                                                                (k + 1, j - 1)]
                            if (self.game_map.player_heading == LaserTankMap.RIGHT): next_pos = [(k - 1 - 1, j),
                                                                                                 (k + 1 - 1,
                                                                                                  j - 1), (
                                                                                                     k + 1 - 1,
                                                                                                     j - 1),
                                                                                                 (k - 1, j - 1)]
                            for position in next_pos:
                                temp_map.player_x = position[0]
                                temp_map.player_y = position[1]
                                temp_map.player_heading = self.game_map.player_heading
                                # ICE,LAND,TELE,FLAG use V(s'), WATER use V(s)
                                instant_cost = temp_map.apply_move('f')
                                if (instant_cost == self.game_map.collision_cost):
                                    estimate_value = estimate_value + (1 - self.game_map.t_success_prob) / 5 * (
                                            instant_cost + temp_map.gamma * \
                                            values[self.game_map.player_x - 1][self.game_map.player_y - 1][
                                                self.game_map.player_heading])
                                elif (instant_cost == self.game_map.game_over_cost):
                                    estimate_value = estimate_value + (
                                            1 - self.game_map.t_success_prob) / 5 * instant_cost
                                else:
                                    estimate_value = estimate_value + (1 - self.game_map.t_success_prob) / 5 * (
                                            instant_cost + temp_map.gamma * \
                                            values[temp_map.player_x - 1][temp_map.player_y - 1][
                                                temp_map.player_heading])
                            values[k - 1][j - 1][i] = estimate_value

        # loop
        # F initial set 0
        # wall and water set 0, because already have collision_cost and game_over_cost
        # above cell do not change V value any more!!!
        count = 1
        converged=False
        while (converged==False):# and current-start<=self.game_map.time_limit):
            count = count + 1
            current = time.time()
            print("time: ", current - start, "max_delta: ", converged, " count: ", count)
            converged=True
            for j in range(1, self.game_map.y_size - 1):
                for k in range(1, self.game_map.x_size - 1):
                    for i in LaserTankMap.DIRECTIONS:
                        if (self.game_map.grid_data[j][k] == LaserTankMap.LAND_SYMBOL
                                or self.game_map.grid_data[j][k] == LaserTankMap.TELEPORT_SYMBOL
                                or self.game_map.grid_data[j][k] == LaserTankMap.ICE_SYMBOL):
                            self.game_map.player_x = j
                            self.game_map.player_y = k
                            self.game_map.player_heading = i
                            current_policy = policy[k - 1][j - 1][i] #!!!different from vi
                            max_value = -math.inf  # choose the action result in the max V value
                            for a in LaserTankMap.MOVES:  # do an action, NO SHOT!
                                if (a == 's'):
                                    continue
                                elif (a == 'l'):
                                    self.game_map.player_x = k
                                    self.game_map.player_y = j
                                    self.game_map.player_heading = i
                                    # if (self.game_map.player_heading==LaserTankMap.UP): direction = LaserTankMap.LEFT
                                    # if (self.game_map.player_heading == LaserTankMap.LEFT): direction = LaserTankMap.DOWN
                                    # if (self.game_map.player_heading == LaserTankMap.DOWN): direction = LaserTankMap.RIGHT
                                    # if (self.game_map.player_heading == LaserTankMap.RIGHT): direction = LaserTankMap.UP
                                    estimate_value = self.game_map.apply_move('l') + self.game_map.gamma * \
                                                     values[self.game_map.player_x - 1][
                                                         self.game_map.player_y - 1][self.game_map.player_heading]
                                    if (max_value < estimate_value):
                                        max_policy = 'l'
                                        max_value = estimate_value
                                elif (a == 'r'):
                                    self.game_map.player_x = k
                                    self.game_map.player_y = j
                                    self.game_map.player_heading = i
                                    # if (self.game_map.player_heading == LaserTankMap.UP): direction = LaserTankMap.RIGHT
                                    # if (self.game_map.player_heading == LaserTankMap.LEFT): direction = LaserTankMap.UP
                                    # if (self.game_map.player_heading == LaserTankMap.DOWN): direction = LaserTankMap.LEFT
                                    # if (self.game_map.player_heading == LaserTankMap.RIGHT): direction = LaserTankMap.DOWN
                                    estimate_value = self.game_map.apply_move('r') + self.game_map.gamma * \
                                                     values[self.game_map.player_x - 1][
                                                         self.game_map.player_y - 1][self.game_map.player_heading]
                                    if (max_value < estimate_value):
                                        max_policy = 'r'
                                        max_value = estimate_value
                                else:  # move forward
                                    self.game_map.player_x = k
                                    self.game_map.player_y = j
                                    self.game_map.player_heading = i
                                    temp_map = self.game_map.make_clone()  # because we want to CHEAT(prob=1) so must have a copy
                                    temp_map.t_success_prob = 1  # CHEAT haha, use in build method
                                    estimate_value = 0
                                    # all the move done on estimate map
                                    # 1st s' success
                                    instant_cost = temp_map.apply_move('f')
                                    if (instant_cost == self.game_map.collision_cost):
                                        estimate_value = estimate_value + self.game_map.t_success_prob * (
                                                instant_cost + temp_map.gamma * \
                                                values[self.game_map.player_x - 1][self.game_map.player_y - 1][
                                                    self.game_map.player_heading])
                                    elif (instant_cost == self.game_map.game_over_cost):  # absorbing state
                                        estimate_value = estimate_value + self.game_map.t_success_prob * instant_cost
                                    else:
                                        estimate_value = estimate_value + self.game_map.t_success_prob * (
                                                instant_cost + temp_map.gamma * \
                                                values[temp_map.player_x - 1][temp_map.player_y - 1][
                                                    temp_map.player_heading])
                                    # 2nd s' not move
                                    estimate_value = estimate_value + (1 - self.game_map.t_success_prob) / 5 * (
                                                temp_map.move_cost + temp_map.gamma * \
                                                values[self.game_map.player_x - 1][self.game_map.player_y - 1][
                                                    self.game_map.player_heading])
                                    # other 4 s'
                                    if(self.game_map.player_heading==LaserTankMap.UP):next_pos=[(k-1,j),(k-1,j+1),(k+1,j),(k+1,j+1)]
                                    if (self.game_map.player_heading == LaserTankMap.DOWN): next_pos = [(k - 1, j-1),(k - 1, j ),(k + 1, j), (k + 1, j-1)]
                                    if (self.game_map.player_heading == LaserTankMap.LEFT): next_pos = [(k+1, j+1),(k , j + 1),(k, j-1),(k +1, j-1)]
                                    if (self.game_map.player_heading == LaserTankMap.RIGHT): next_pos = [(k  -1, j-1),(k , j - 1),(k, j+1),(k -1, j+1)]
                                    for position in next_pos:
                                        temp_map.player_x = position[0]
                                        temp_map.player_y = position[1]
                                        temp_map.player_heading = self.game_map.player_heading
                                        # ICE,LAND,TELE,FLAG use V(s'), WATER use V(s)
                                        instant_cost = temp_map.apply_move('f')
                                        if (instant_cost == self.game_map.collision_cost):
                                            estimate_value = estimate_value + (1 - self.game_map.t_success_prob) / 5 * (
                                                        instant_cost + temp_map.gamma * \
                                                        values[self.game_map.player_x - 1][self.game_map.player_y - 1][
                                                            self.game_map.player_heading])
                                        elif (instant_cost == self.game_map.game_over_cost):
                                            estimate_value = estimate_value + (
                                                        1 - self.game_map.t_success_prob) / 5 * instant_cost
                                        else:
                                            estimate_value = estimate_value + (1 - self.game_map.t_success_prob) / 5 * (
                                                        instant_cost + temp_map.gamma * \
                                                        values[temp_map.player_x - 1][temp_map.player_y - 1][
                                                            temp_map.player_heading])
                                    if (max_value < estimate_value):
                                        max_policy = 'f'
                                        max_value = estimate_value
                            values[k - 1][j - 1][i] = max_value
                            policy[k - 1][j - 1][i] = max_policy
                            if(current_policy!=max_policy):converged=False #!!!different from vi
            current = time.time()

        # store the computed values and policy
        self.values = values
        self.policy = policy

    def get_offline_value(self, state):
        """
        Get the value of this state.
        :param state: a LaserTankMap instance
        :return: V(s) [a floating point number]
        """

        #
        # TODO
        # Write code to return the value of this state based on the stored self.values
        #
        # You can assume that either run_value_iteration( ) or run_policy_iteration( ) has been called before this
        # method is called.
        #
        # When this method is called, you are allowed up to 1 second of compute time.
        #
        return self.values[state.player_x-1][state.player_y-1][state.player_heading]
        pass

    def get_offline_policy(self, state):
        """
        Get the policy for this state (i.e. the action that should be performed at this state).
        :param state: a LaserTankMap instance
        :return: pi(s) [an element of LaserTankMap.MOVES]
        """

        #
        # TODO
        # Write code to return the optimal action to be performed at this state based on the stored self.policy
        #
        # You can assume that either run_value_iteration( ) or run_policy_iteration( ) has been called before this
        # method is called.
        #
        # When this method is called, you are allowed up to 1 second of compute time.
        #
        return self.policy[state.player_x-1][state.player_y-1][state.player_heading]
        pass

    def get_mcts_policy(self, state):
        """
        Choose an action to be performed using online MCTS.
        :param state: a LaserTankMap instance
        :return: pi(s) [an element of LaserTankMap.MOVES]
        """

        #
        # TODO
        # Write your Monte-Carlo Tree Search implementation here.
        #
        # Each time this method is called, you are allowed up to [state.time_limit] seconds of compute time - make sure
        # you stop searching before this time limit is reached.
        #
        """ node class and method for mcts """
        # Only store state node, action list with weight is inside the state node
        # I dont care edges between states, apply_move will tell me next state with uncertainty anyway
        """
        idea: a set of state node with no explict edges
              from root to a leaf s
              from s choose an action with sampling strategy
              use apply_move(action) get s' with uncertainty
              from s' use ??? to simulate (from s only the first action is choosen by sampling, all rest are simulate)
              renew values of that action in s, till root
              !!! there is no parent or children state node, because of uncertainty. If so will have duplicate
              every iter will record the nodes it visited for backpropagate
        """
        class StateNode:
            def __init__(self, map, player_x, player_y, heading):
                self.map = map #grid_data
                self.player_x = player_x
                self.player_y = player_y
                self.heading = heading
                self.children=[['s',0,0,0],['f',0,0,0],['l',0,0,0],['r',0,0,0]] # action,win,loss,n  (n-win-loss= # no result)

            def __eq__(self, other):
                s1=self
                s2=other
                if not isinstance(s1, StateNode) or not isinstance(s2, StateNode):
                    return False
                if (s1.player_y!=s2.player_y or s1.player_x!=s2.player_x or s1.heading!=s2.heading):
                    return False
                for i in range(len(s1.game_map)):
                    for j in range(len(s1.game_map[0])):
                        for k in range(len(s1.game_map[0][0])):
                            if not s1.game_map[i][j][k]!=s2.game_map[i][j][k]:
                                return False
                self.match = s2
                """trick! https://code.activestate.com/recipes/499299/"""
                return True

            def __hash__(self):
                tuple_list=[]
                tuple_list.extend([self.player_x,self.player_y,self.heading])
                for col in self.map:
                    tuple_list.extend(col)
                return hash(tuple(tuple_list)) # tuple() must be an iterable var
                """https://stackoverflow.com/questions/4950155/objects-as-keys-in-python-dictionaries"""

        def is_newS(node):
            # if all of action, n=0, means no simulation for this node. Then is_newS
            if (node.children[0][3]==0 and node.children[1][3]==0 and node.children[2][3]==0 and node.children[3][3]==0):
                return True
            return False

        def cal_prob(a_tuple):
            """
            calculate weight for an action
            :param a_tuple: input: eg ('s',0,0,0) a,win,loss,n
            :return: weight value
            """
            w=a_tuple[1]
            a=a_tuple[0]
            l=a_tuple[2]
            n=a_tuple[3]
            return w/n+C*math.sqrt(math.log(N,math.e)/n)

        def manipulate_global_list(node):
            """
            path[] and v_list controller
            path[](can 'duplicate', save path)
            v_list(no duplicate, save prob of action[]):
            return equivalent star in v_list, or add star into v_list
            """
            # check wether already in the v_list
            if node in v_list:
                # if yes: node=v_list.get_node
                return node.match
            else:
                # if no: node store into v_list and does not change
                v_list.add(node)
                return node

        def get_action_prob(node):
            """
            select an action of a node by PROB
            :param node: s
            :return: an action
            """
            prob = []
            prob[0] = cal_prob(node.children[0])
            prob[1] = cal_prob(node.children[1])
            prob[2] = cal_prob(node.children[2])
            prob[3] = cal_prob(node.children[3])
            actions=['s','f','l','r']
            totals = []
            running_total = 0
            # choose an action by weight
            for i in prob:
                running_total += i
                totals.append(running_total)
            rnd = random.random() * running_total
            for i, total in enumerate(totals):
                if rnd < total:
                    return actions[i]

        def get_action_Astrategy(node):
            """
            1.highest prob: turn direct to the flag, forward to the flag
            2.other 2 prob: other 2 actions
            almost no rand, just direct to the flag
            :param node:
            :return: an action
            """
            x_coor=node.player_x-self.F_x
            y_coor=(-node.player_y)-(-self.F_y)
            if(y_coor<0 and node.heading==0):action='f'
            if (y_coor > 0 and node.heading==1):action='f'
            if (x_coor>0 and node.heading==2):action='f'
            if(x_coor<0 and node.heading==3):action='f'
            if(x_coor==0):
                if(y_coor>0):
                    if(node.heading==2):action='l'
                    if (node.heading == 3): action = 'r'
                    if (node.heading == 0): action = 'r'
                if(y_coor<0):
                    if (node.heading == 2): action = 'r'
                    if (node.heading == 3): action = 'l'
                    if (node.heading == 1): action = 'r'
            if(y_coor==0):
                if (x_coor > 0):
                    if (node.heading == 0): action = 'l'
                    if (node.heading == 1): action = 'r'
                    if (node.heading == 3): action = 'r'
                if (x_coor < 0):
                    if (node.heading == 0): action = 'r'
                    if (node.heading == 1): action = 'l'
                    if (node.heading == 2): action = 'r'
            if(x_coor>0 and y_coor>0):
                if (node.heading == 0): action = 'l'
                if (node.heading == 3): action = 'r'
            if (x_coor > 0 and y_coor < 0):
                if (node.heading == 1): action = 'r'
                if (node.heading == 3): action = 'l'
            if (x_coor < 0 and y_coor < 0):
                if (node.heading == 1): action = 'l'
                if (node.heading == 2): action = 'r'
            if (x_coor < 0 and y_coor > 0):
                if (node.heading == 0): action = 'r'
                if (node.heading == 2): action = 'l'
            return action
            #prob 1,0,0

        def set_mct_map(node):
            """
            use node set mct_map(global LASERTANK class), usually for apply_move/check_die_star?
            :param node:
            :return:
            """
            mct_game_map.grid_data = node.map
            mct_game_map.player_heading = node.heading
            mct_game_map.player_y = node.player_y
            mct_game_map.player_x = node.player_x

        def get_nextS_prob(node):
            """
            from s, choose action by PROB!!!
            then get s' by apply_move with uncertainty
            ONLY FOR selection_expansion, because we put action in to path(node,action)
            :param node: s
            :return: s'
            """
            # from s, choose action by prob, get s'
            action = get_action_prob(node)
            path[-1][1]=action
            # do action and get s'
            set_mct_map(node)
            result=mct_game_map.apply_move(action)
            node = StateNode(mct_game_map.grid_data, mct_game_map.player_x, mct_game_map.player_y,
                         mct_game_map.player_heading)
            return node

        def get_nextS_action(node,action):
            """
            from s, choose action by ACTION!!!
            then get s' by apply_move with uncertainty
            :param node: s
            :return: s'
            """
            set_mct_map(node)
            result = mct_game_map.apply_move(action)
            node = StateNode(mct_game_map.grid_data, mct_game_map.player_x, mct_game_map.player_y,
                             mct_game_map.player_heading)
            return node

        def clone_grid(grid):
            """2D grid clone"""
            grid2=[len(grid)][len(grid[0])]
            for i in len(grid)-1:
                for j in len(grid[0])-1:
                    grid2[i][j]=grid[i][j]
            return grid2


        def compare_grid(g1,g2):
            """2D grid compare"""
            for i in len(g1)-1:
                for j in len(g1[0])-1:
                    if (g1[i][j]!=g2[i][j]): return False
            return True


        def get_nextS_Astrategy(node):
            """
            shot first if useful, if shot no use then A* with random.
            Because it is simple map!!! otherwise not work
            :param node:
            :return:
            """
            set_mct_map(node)
            # action should closer to the flag with random
            action=get_action_Astrategy(node) # no shot!!
            result = mct_game_map.apply_move(action)
            node = StateNode(mct_game_map.grid_data, mct_game_map.player_x, mct_game_map.player_y,
                              mct_game_map.player_heading)
            return node


        def selection_expand():
            """
            from root, select path(selection); till a new_S (expand)
            :return: a new_S node (a node with no action has a weight)
            """
            temp_node=root
            flag_newS=is_newS(temp_node) #not necessary new, but not exploit any of its actions yet
            node=manipulate_global_list(temp_node)
            path.append((node,None))
            while (flag_newS==False):
                temp_node=get_nextS_prob(node) #select an action and apply_move and get s'
                flag_newS=is_newS(temp_node) # analysis whether s' is new_S
                node = manipulate_global_list(temp_node)
                path.append(node, None)
            return node

        def is_absorbing_state(node):
            """
            absorbing state?
            :param node:
            :return: -1 die, +1 flag, 0 not die_star
            """
            set_mct_map(node)
            if(mct_game_map.cell_is_game_over(mct_game_map.player_y,mct_game_map.player_x)):return -1
            if(mct_game_map.is_finished()):return 1
            return 0

        def simulate_Astrategy(node,n):
            """shot first then 'f' then Astrategy"""
            count=0
            shot_count=0
            while (count <= n):  # after n no result then give up
                count = count + 1
                if shot_count <= 6:
                    shot_count =shot_count+1
                    count=count-1
                    set_mct_map(node)
                    # try shot first
                    result = mct_game_map.apply_move('s')
                    node = StateNode(mct_game_map.grid_data, mct_game_map.player_x, mct_game_map.player_y,
                                     mct_game_map.player_heading)
                elif count<=2:
                    set_mct_map(node)
                    # try shot first
                    result = mct_game_map.apply_move('f')
                    node = StateNode(mct_game_map.grid_data, mct_game_map.player_x, mct_game_map.player_y,
                                     mct_game_map.player_heading)
                else:  # following actions
                    node = get_nextS_Astrategy(node)
                # check absorbing state
                absorbing = is_absorbing_state(node)
                if (absorbing == -1): return -1
                if (absorbing == 1): return 1
            return -1  # can be 0, out of fuel

        def one_simulate(node,action):
            """
            simulation start from node, first action controlled, the rest actions by strategy
            :param node:
                   action: first action
            :return: -1 die, 0 out of fuel(not using), 1 flag
            """
            # deepth control, use manhatton*2 because it is easy map, tank need not turn back after shot
            n=1.2*(abs(node.player_x-self.F_x)+abs(node.player_y-self.F_y))
            node = get_nextS_action(node, action)
            absorbing = is_absorbing_state(node)
            if (absorbing == -1): return -1
            if (absorbing == 1): return 1

            res=simulate_Astrategy(node,n)
            return res


        def simulate(initial_node):
            """
            FIRST, check whether initial_node is game_over or flag_cell
            all the action of node simulate n times and get win(Flag),loss(Die),n value
            :param node: a new_S node
            :return: result_list: a list, eg [('s',0,0,0),('f',0,0,0),('l',0,0,0),('r',0,0,0)] a,win,loss,n
                    or -1,1 if the initial_node is already absorbing
            """
            absorbing=is_absorbing_state(initial_node)
            # I dont manipulate global v_list here, only delete last ele of path[] as it is a absorbing(die_star)
            # as absorbing(die_star)'s action always 0,
            # and will not expand v_list as when add already check duplicate, so no need to be that well structured
            if(absorbing==-1): path.pop(-1);return -1
            if(absorbing==1): path.pop(-1);return 1
            n=50
            actions=['s','f','l','r']
            result_list=[['s',0,0,n],['f',0,0,n],['l',0,0,n],['r',0,0,n]]
            for action in actions:
                for i in range(n):
                    res = one_simulate(initial_node,action)
                    if res==1: result_list[actions.index(action)][1]=result_list[actions.index(action)][1]+1
                    if res==-1: result_list[actions.index(action)][2]=result_list[actions.index(action)][2]+1
            return result_list

        def backpropagate_sub(win, loss, n):
            """
            all the node in the path[], update with 3 inputs
            """
            node_action = path.pop(-1)
            temp_node=node_action[0];action=node_action[1]
            backpropagate_updateOne(temp_node, action, win, loss, n)

        def backpropagate_updateOne(temp_node, action, win, loss, n):
            """
            update one node one action with 3 inputs
            2.find relative node in v_list, 3.update action
            """
            node=manipulate_global_list(temp_node)
            for index, item in enumerate(node.children):
                if action==item[0]:
                    node.children[index][1]=node.children[index][1]+win
                    node.children[index][2]=node.children[index][2]+loss
                    node.children[index][3]=node.children[index][3]+n




        def backpropagate(result_list):
            """
            the last one if not absorbing need all actions weight update !!!
            the previous ones only one action weight need update
            1.from path[] get (node,action), 2.find relative node in v_list, 3.update action
            output: change all the acenstor's w and n (in action list) in the path[]
            :param result_list: -1/1/list eg [('s',0,0,0),('f',0,0,0),('l',0,0,0),('r',0,0,0)] a,win,loss,n
                   path[]: global
            :return: None
            """
            m=50 # the same in simulator
            if(result_list==1):
                backpropagate_sub(m*4,0,m*4)
            elif(result_list==-1):
                backpropagate_sub(0, m*4, m*4)
            else:
                #the last node update
                node_action=path.pop(-1) #(node,action)
                temp_node=node_action[0]
                for i in result_list:
                    action=i[0];win=i[1];loss=i[2];n=i[3]
                    backpropagate_updateOne(temp_node, action, win, loss, n)
                #other nodes update
                total_win=0;total_loss=0;total_simu=0
                for i in result_list:
                    total_win=total_win+i[1]
                    total_loss = total_loss + i[2]
                    total_n = total_simu + i[3]
                backpropagate_sub(total_win, total_loss, total_n)


        """======================methods above main below========================"""

        #initialize
        print("===================================================")
        C=math.sqrt(2)
        N=0 # num of total simulation
        mct_game_map=state.make_clone()
        root = StateNode(mct_game_map.grid_data,mct_game_map.player_x,mct_game_map.player_y,mct_game_map.player_heading)

        # i guess should use hash to determine whether 2 node(star) are the same
        # the action[] weight can be different
        # self.map,self.player_x,self.player_y,self.heading same => same node(star)
        v_list = set()

        """
        I dont care leaf, if apply_move get a s' that is not in v_list, then s is 'leaf'.
        'leaf' can means a StateNode with some action that has not been exploited
        BUT in my implementation, 'leaf' means a StateNode that has s' has not been exploited
        selection combine with expansion: because only when s' is not in v_list, I can know that s is leaf choosen
        simulator: from node_S, all the action do simulate n times, get 0,1s
        back propagate: renew - children[actions tuples] in StateNodes
        """
        start=time.time()
        current=start
        count=0
        while(current-start<mct_game_map.time_limit and count<1):
            count=count+1
            #initial
            path=[] #store the (state,action) in one iter for back propagate, if the expanding node, action=None

            # selection(select a path from root to one curent leaf)
            # input:root node
            # output:a new_S node
            # ============================================
            # expand (till Flag)
            # input:a leaf node
            # output: select an action and get a s' with uncertainty
            node = selection_expand()
            """ IMPORTANT, this node can be flag or die
            #   So MUST check it(decide whether delete from path[] and back propagate) 
            #   before this attri disappear """

            """python: if an attri is defined in global
            1. if same name attri is not assign value in def, then global 2. if same name attri is also assigned in dfe, local"""

            # node is a new_S now, new_S can be game_over_cell or flag_cell, no worries, in simulator I will check and delete from path[] if is
            # path[] is from root to leaf now, (node,action), the last node is (node,None), the last node maybe die_star

            """======================================================================================="""
            # simulator
            # simulate all the action of new_S, the first action is 's,f,l,r', the folow actions use A* (prob=1)
            # input:s'
            # output: results 1 node is flag/-1 node is die/result_list, n times for all the actions in s'
            #         eg [('s',0,0,0),('f',0,0,0),('l',0,0,0),('r',0,0,0)] a,win,loss,n
            result_list=simulate(node)

            # result_list is the outcome of simulate, eg [('s',0,0,0),('f',0,0,0),('l',0,0,0),('r',0,0,0)] a,win,loss,n
            # path[] has delete the last die_star, all the (node,action) need to be update

            """========================================================================================"""
            # backprobagate
            # the last one if not absorbing need all actions weight update !!!
            # the previous ones only one action weight need update
            # from path[] get (node,action), find relative node in v_list, update action
            # input: path[] and result_list
            # output: change all the acenstor's w and n (in action list) in the path[]
            backpropagate(result_list)

            # now all the node in v_list that is in path is updated

            """========================================================================================"""

            current=time.time()
            print("count: ",count," time: ",time)

        #return one the root action with highest weight, online search! we only need one step
        node=manipulate_global_list(root) #find root in the v_list
        actions=node.children
        max_action=''
        max_prob=0
        for action in actions: #choose action with most n
            if max_prob<action[3]:
                max_prob=action[3]
                max_action=action[0]
        return max_action  # return the largest action








