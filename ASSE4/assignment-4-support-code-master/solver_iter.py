import time
import random
from laser_tank import LaserTankMap, DotDict

"""
Template file for you to implement your solution to Assignment 4. You should implement your solution by filling in the
following method stubs:
    train_q_learning()
    train_sarsa()
    get_policy()
    
You may add to the __init__ method if required, and can add additional helper methods and classes if you wish.

To ensure your code is handled correctly by the autograder, you should avoid using any try-except blocks in your
implementation of the above methods (as this can interfere with our time-out handling).

COMP3702 2020 Assignment 4 Support Code
"""


class Solver:

    def __init__(self):
        """
        Initialise solver without a Q-value table.
        """

        #
        # TODO
        # You may add code here if you wish (e.g. define constants used by both methods).
        #
        # The allowed time for this method is 1 second.
        #

        self.q_values = None

    def epsilon_greedy(self,max_action,actions):
        actions_list=list(actions)
        epsilon=0.8
        weights=[]
        for action in actions_list:
            weights.append(1-epsilon)
            if action==max_action:
                weights[-1]=epsilon
        totals = []
        running_total = 0
        # choose an action by weight
        for i in weights:
            running_total += i
            totals.append(running_total)
        rnd = random.random() * running_total
        for i, total in enumerate(totals):
            if rnd < total:
                return actions_list[i]

    def get_action(self, map, q_values):
        action_values_pair=q_values.get(hash(map))
        weights=action_values_pair.values()
        actions=action_values_pair.keys()
        max_key = max(action_values_pair, key=action_values_pair.get)
        action=self.epsilon_greedy(max_key,actions)
        return action


    def add_state(self, map,q_values):
        """
        if hash(map) not in q_values, add into and set actions to 0
        :return:
        """
        key=hash(map)
        if not key in q_values:
            q_values.update({key:{"s":0,"f":0,"l":0,"r":0}}) #make sure the update also happen in main function

    def max_Q(self, simulator, q_values):
        """
        get max Q with max a of the state(simulator)
        :param q_values:
        :return:
        """
        action_values_dict=q_values.get(hash(simulator))
        all_values = action_values_dict.values()
        max_value = max(all_values)
        return max_value

    def next_Q(self, simulator, q_values):
        """
        get Q with strategy a of the state(simulator)
        :param simulator:
        :param q_values:
        :return: action,next_q_value
        """
        action=self.get_action(simulator, q_values)
        Q_value=q_values.get(hash(simulator)).get(action)
        return action,Q_value

    def train_q_learning(self, simulator,iter,alpha):
        """
        Train the agent using Q-learning, building up a table of Q-values.
        :param simulator: A simulator for collecting episode data (LaserTankMap instance)
        """

        # Q(s, a) table
        # suggested format: key = hash(state), value = dict(mapping actions to values)
        q_values = {}

        #
        # TODO
        # Write your Q-Learning implementation here.
        #
        # When this method is called, you are allowed up to [state.time_limit] seconds of compute time. You should
        # continue training until the time limit is reached.
        #

        #initialization, later evaluation
        temp_map=simulator.make_clone() # temp_map used to apply move, simulator is the origin map
        # maybe no need, because we have reset
        alpha=alpha

        count=0
        # iteration episode, every episode start from initial s
        key=hash(simulator)
        q_values.update({key:{"s":0,"f":0,"l":0,"r":0}})
        start = time.time()
        end = start
        while((end-start)<simulator.time_limit and count<=iter):
            count=count+1
            #print("count: ",count," time: ",end-start)
            #set up initial s
            simulator.reset_to_start();
            #iter until terminal state
            flag=False
            while(flag==False):
                # choose an action
                s_key=hash(simulator)
                action=self.get_action(simulator,q_values)
                # now simulator=s

                # apply move get s'
                reward,flag=simulator.apply_move(action) #flag:episode finish die/flag
                # now simulator=s'

               # analyze whether has key

                # if not add key and set all action=0
                self.add_state(simulator, q_values) #whether the function start correctly

                # get max Q(s',a')
                max_q_nextState= self.max_Q(simulator, q_values)

                # update Q
                current_q=q_values.get(s_key).get(action)
                updated_q=current_q+alpha*(reward+simulator.gamma*max_q_nextState-current_q)
                q_values.get(s_key)[action]=updated_q

            end=time.time()

        # store the computed Q-values
        self.q_values = q_values

    def train_sarsa(self, simulator,iter,alpha):
        """
        Train the agent using SARSA, building up a table of Q-values.
        :param simulator: A simulator for collecting episode data (LaserTankMap instance)
        """

        # Q(s, a) table
        # suggested format: key = hash(state), value = dict(mapping actions to values)
        q_values = {}

        #
        # TODO
        # Write your SARSA implementation here.
        #
        # When this method is called, you are allowed up to [state.time_limit] seconds of compute time. You should
        # continue training until the time limit is reached.
        #

        alpha = alpha

        count = 0
        # iteration episode, every episode start from initial s
        key = hash(simulator)
        q_values.update({key: {"s": 0, "f": 0, "l": 0, "r": 0}})
        start = time.time()
        end = start
        while ((end - start) < simulator.time_limit and count<=iter):
            count = count + 1
            # print("count: ",count," time: ",end-start)
            # set up initial s
            simulator.reset_to_start();
            # choose an action
            s_key = hash(simulator)
            action = self.get_action(simulator, q_values)
            # iter until terminal state
            flag = False
            while (flag == False):
                # now simulator=s
                s_key = hash(simulator)
                # apply move get s'
                reward, flag = simulator.apply_move(action)  # flag:episode finish die/flag
                # now simulator=s'

                # analyze whether has key
                # if not, add key and set all action=0
                self.add_state(simulator, q_values)  # whether the function start correctly

                # get Q(s',a') by choose a' with strategy
                next_action,q_nextState = self.next_Q(simulator, q_values)

                # update Q
                current_q = q_values.get(s_key).get(action)
                updated_q = current_q + alpha * (reward + simulator.gamma * q_nextState - current_q)
                q_values.get(s_key)[action] = updated_q

                # s<-s',a<-a'
                action=next_action

            end = time.time()


        # store the computed Q-values
        self.q_values = q_values

    def get_policy(self, state):
        """
        Get the policy for this state (i.e. the action that should be performed at this state).
        :param state: a LaserTankMap instance
        :return: pi(s) [an element of LaserTankMap.MOVES]
        """

        #
        # TODO
        # Write code to return the optimal action to be performed at this state based on the stored Q-values.
        #
        # You can assume that either train_q_learning( ) or train_sarsa( ) has been called before this
        # method is called.
        #
        # When this method is called, you are allowed up to 1 second of compute time.
        #
        action_values_pair=self.q_values.get(hash(state))
        if not action_values_pair:
            return 's'
        max_key = max(action_values_pair, key=action_values_pair.get)
        return max_key






