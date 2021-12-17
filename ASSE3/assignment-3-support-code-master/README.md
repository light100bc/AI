# Assignment 3 Support Code

This is the support code for COMP3702 2019 Assignment 3.

The following files are provided:

**laser_tank.py**

This file contains a class representing Laser Tank game map. This is similar to the file provided in Assignment 1, but has a number of important distinctions.

The LaserTankMap class has the following instance variables:
- x_size
- y_size
- t_success_prob
- t_error_prob
- gamma
- epsilon
- method
- benchmark
- time_limit
- initial_seed
- grid_data
- player_x
- player_y
- player_heading
- flag_x
- flag_y

You should make use of t_success_prob (and/or t_error prob) in implementing your transition function.

You should make use of gamma (the discount factor) and epsilon (the termination criteria) in your algorithm implementations.

You can use time_limit as an additional terminating condition (to make you code stop iterating before the time limit is reached).

method is handled by the tester script - you don't need to select which algorithm will be used in your code, just fill in the method stubs in solver.py.

This class contains a number of functions which will be useful in developing your solver.

The static method
~~~~~
LaserTankMap.process_input_file(filename)
~~~~~
can be used to parse input files (testcases) and produce a LaserTankMap instance based on the input file.

The instance method
~~~~~
apply_move(self, move)
~~~~~
applies a *non-deterministic* action to the Laser Tank game map, changing it's state. When the LaserTankMap.MOVE_FORWARD action is selected, the probability of ending in the square directly ahead is given by t_success_prob (an instance variable of the LaserTankMap object). All other directions (forward-left, forward-right, left, right and no-change) have equal probability.

Note that this method will mutate the internal variables of the class. To deep copy, make use of the
~~~~~
make_clone(self)
~~~~~
method.

This method returns a reward value (move_cost, collision_cost, game_over_cost or goal_reward - each of which are instance variables of a LaserTankMap instance) depending on the result of the applied move.

The instance method
~~~~~
is_finished(self)
~~~~~
tells you whether the state of this game map matches the goal state (i.e. the player's tank is on the flag tile).

You can run this file directly to launch an interactive game of Laser Tank. e.g:
~~~~~
$ python laser_tank.py testcases/<testcase_name>.txt
~~~~~
Press W to move forward, D and A to turn clockwise and counter-clockwise respectively, and spacebar to shoot the laser. Try this to get a feel for the rules and mechanics of the game.

**tester.py**

Use this script to test the policy produced by your code. e.g:
~~~~~
$ python tester.py testcases/<testcase_name>.txt
~~~~~
This will indicate the total reward received over a simulated episode compared to the benchmark. Our autograder will make use of a copy of this tester script. To make sure your code is graded correctly, make sure your output files pass this tester.

~~~~~
get_optimal_number_of_steps(filename)
get_time_limit(filename)
~~~~~

**policy_visualiser.py**

An animated version of tester which shows each step your agent takes in a simulated episode. Use the same way as tester.

**solver.py**

A template for you to write your solution.

You should implement your solution by filling in the following method stubs:
- run_value_iteration()
- run_policy_iteration()
- get_offline_value()
- get_offline_policy()
 - get_mcts_policy()
    
You may add to the init method if required, and can add additional helper methods and classes if you wish.

To ensure your code is handled correctly by the autograder, you should avoid using any try-except blocks in your implementation of the above methods (as this can interfere with our time-out handling).

Do not rename this file or the Solver class.

**testcases**

A set of testcases for you to evaluate your solution. Note that VI and PI testcases contain only LAND, WATER, OBSTACLE, ICE, TELEPORT and FLAG symbols, while MCTS testcases can contain any symbol.

