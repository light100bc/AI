import copy
import numpy as np
import random
import time

# Directions
UP = 0
DOWN = 1
LEFT = 2
RIGHT = 3

OBSTACLES = [(1, 1)]
EXIT_STATE = (-1, -1)

class Grid:

    def __init__(self):
        self.x_size = 4
        self.y_size = 3
        self.p = 0.8
        self.actions = [UP, DOWN, LEFT, RIGHT]
        self.rewards = {(3, 1): -100, (3, 2): 1}
        self.discount = 0.9

        self.states = list((x, y) for x in range(self.x_size) for y in range(self.y_size))
        self.states.append(EXIT_STATE)
        for obstacle in OBSTACLES:
            self.states.remove(obstacle)

    def attempt_move(self, s, a):
        # Check absorbing state
        if s == EXIT_STATE:
            return s

        # Default: no movement
        result = s 

        # Check borders
        """
        Write code here to check if applying an action 
        keeps the agent with the boundary
        """

        # Check obstacle cells
        """
        Write code here to check if applying an action 
        moves the agent into an obstacle cell
        """

        return result

    def stoch_action(self, a):
        # Stochasitc actions probability distributions
        if a == RIGHT: 
            stoch_a = {RIGHT: self.p , UP: (1-self.p)/2, DOWN: (1-self.p)/2}
        if a == UP:
            stoch_a = {UP: self.p , LEFT: (1-self.p)/2, RIGHT: (1-self.p)/2}
        if a == LEFT:
            stoch_a = {LEFT: self.p , UP: (1-self.p)/2, DOWN: (1-self.p)/2}
        if a == DOWN:
            stoch_a = {DOWN: self.p , LEFT: (1-self.p)/2, RIGHT: (1-self.p)/2}
        return stoch_a

    def get_reward(self, s):
        if s == EXIT_STATE:
            return 0

        if s in self.rewards:
            return self.rewards[s]
        else:
            return 0

class ValueIteration:
    def __init__(self, grid):
        self.grid = Grid()
        self.values = {state: 0 for state in self.grid.states}

    def next_iteration(self):
        new_values = dict()
        """
        Write code here to imlpement the VI value update
        Iterate over self.grid.states and self.grid.actions
        Use stoch_action(a) and attempt_move(s,a)
        """
        self.values = new_values

    def print_values(self):
        for state, value in self.values.items():
            print(state, value)


class PolicyIteration:
    def __init__(self, grid):
        self.grid = Grid()
        self.values = {state: 0 for state in self.grid.states}
        self.policy = {pi: RIGHT for pi in self.grid.states}
        self.r = [0 for s in self.grid.states]
        idx = 0
        for state in self.grid.states: 
            if state in self.grid.rewards.keys(): 
                self.r[idx] = self.grid.rewards[state]
            idx = idx+1
        print('r is ', self.r)
        
    def policy_evaluation(self):
        P_pi = np.zeros((len(self.grid.states),len(self.grid.states)), dtype=np.float64)
        """
        Write code to generate the state transition probability matrix 
        under a policy pi here; this is matrix P_pi initialised above
        Ensure the Pi_p index order is consistent with r 
        """ 
        
        A = np.identity(len(self.grid.states)) - self.grid.discount*P_pi
        v = np.linalg.solve(A, self.r)
        self.values = {state: v for state in self.grid.states}

    def policy_improvement(self):
        """
        Write code to extract the best policy for a given value function here
        """ 
        return

    def convergence_check(self):
        """
        Write code to check if PI has converged here
        """   
        return
        
    def print_values(self):
        for state, value in self.values.items():
            print(state, value)
    
    def print_policy(self):
        for state, policy in self.policy.items():
            print(state, policy)


if __name__ == "__main__":
    grid = Grid
    vi = ValueIteration(grid)

    start = time.time()
    print("Initial values:")
    vi.print_values()
    print()

    for i in range(max_iter):
        vi.next_iteration()
        print("Values after iteration", i + 1)
        vi.print_values()
        print()

    end = time.time()
    print("Time to copmlete", max_iter, "VI iterations")
    print(end - start)
    