import sys
import time

from problem_spec import ProblemSpec
from robot_config import write_robot_config_list_to_file, make_robot_config_from_ee2
from tester import *
from visualiser import Visualiser
import random
import math

"""
Template file for you to implement your solution to Assignment 2. Contains a class you can use to represent graph nodes,
and a method for finding a path in a graph made up of GraphNode objects.

COMP3702 2020 Assignment 2 Support Code
"""


class GraphNode:
    """
    Class representing a node in the state graph. You should create an instance of this class each time you generate
    a sample.
    """

    def __init__(self, spec, config):
        """
        Create a new graph node object for the given config.

        Neighbors should be added by appending to self.neighbors after creating each new GraphNode.

        :param spec: ProblemSpec object
        :param config: the RobotConfig object to be stored in this node
        """
        self.spec = spec
        self.config = config
        self.neighbors = []

    def __eq__(self, other):
        return test_config_equality(self.config, other.config, self.spec)

    def __hash__(self):
        return hash(tuple(self.config.points))

    def get_successors(self):
        return self.neighbors

    @staticmethod
    def add_connection(n1, n2):
        """
        Creates a neighbor connection between the 2 given GraphNode objects.

        :param n1: a GraphNode object
        :param n2: a GraphNode object
        """
        n1.neighbors.append(n2)
        n2.neighbors.append(n1)


def find_graph_path(spec, init_node):
    """
    This method performs a breadth first search of the state graph and return a list of configs which form a path
    through the state graph between the initial and the goal. Note that this path will not satisfy the primitive step
    requirement - you will need to interpolate between the configs in the returned list.

    You may use this method in your solver if you wish, or can implement your own graph search algorithm to improve
    performance.

    :param spec: ProblemSpec object
    :param init_node: GraphNode object for the initial configuration
    :return: List of configs forming a path through the graph from initial to goal
    """
    # search the graph
    init_container = [init_node]

    # here, each key is a graph node, each value is the list of configs visited on the path to the graph node
    init_visited = {init_node: [init_node.config]}

    while len(init_container) > 0:
        current = init_container.pop(0)

        if test_config_equality(current.config, spec.goal, spec):
            # found path to goal
            return init_visited[current]

        successors = current.get_successors()
        for suc in successors:
            if suc not in init_visited:
                init_container.append(suc)
                init_visited[suc] = init_visited[current] + [suc.config]

    return None



def main():
# def main(arglist):
#     input_file = arglist[0]
#     output_file = arglist[1]

    input_file="testcases/3g2_m2.txt"
    output_file ="res.txt"
    spec = ProblemSpec(input_file)

    init_node = GraphNode(spec, spec.initial)
    goal_node = GraphNode(spec, spec.goal)

    steps = []

    #
    #
    # Code for your main method can go here.
    #
    # Your code should find a sequence of RobotConfig objects such that all configurations are collision free, the
    # distance between 2 successive configurations is less than 1 primitive step, the first configuration is the initial
    # state and the last configuration is the goal state.
    #
    #
    """====================sampling========================"""
    def points_distance(p1,p2,name="euclidean"):
       if name=="euclidean": return math.sqrt(math.pow(p1[0]-p2[0],2)+math.pow(p1[1]-p2[1],2))



    def get_mid_c(c1,c2):
        if(c1.ee1_grappled!=c2.ee1_grappled):return False
        ee1_delta = []  # total movement of each arm, a list of delta
        ee2_delta = []
        rand_lengths=[]
        rand_angles=[]
        # create p_mid
        if (c1.ee1_grappled == True):
            for i in range(spec.num_segments):
                an_angle=(c2.ee1_angles[i] - c1.ee1_angles[i])/2
                ee1_delta.append(an_angle)
                rand_lengths.append((c2.lengths[i] + c1.lengths[i])/2)
                rand_angles.append(c1.ee1_angles[i]+ee1_delta[i])
            c_mid = make_robot_config_from_ee1(c1.points[0][0], c1.points[0][1],
                                                rand_angles, rand_lengths, ee1_grappled=True)
        else:
            for i in range(spec.num_segments):
                an_angle=(c2.ee2_angles[i] - c1.ee2_angles[i])/2
                ee2_delta.append(an_angle)
                rand_lengths.append((c2.lengths[i] + c1.lengths[i]) / 2)
                rand_angles.append(c1.ee2_angles[i] + ee2_delta[i])
            c_mid = make_robot_config_from_ee2(c1.points[-1][0], c1.points[-1][1],
                                                rand_angles, rand_lengths, ee2_grappled=True)
        return c_mid



    def inD_sampling(config,d):
        """
        d should be manhatton(change one dim a bit) or euclidean(change all dim a SLIGHT)???
        :param config:
        :param d:
        :return:a config
        """

        rand_angles = []
        rand_lengths = []

        if (config.ee1_grappled==True):
            for i in range(spec.num_segments):
                rand_d = random.uniform(-d, d)
                delt_angle = Angle(radians=rand_d)
                an_angle = delt_angle + config.ee1_angles[i]
                rand_angles.append(an_angle)
                is_good_length=False
                while is_good_length==False:
                    rand_d = random.uniform(-d, d)
                    a_length=config.lengths[i]+rand_d*d/1.3*(spec.max_lengths[i]-spec.min_lengths[i])
                    if (a_length<=spec.max_lengths[i] and a_length>=spec.min_lengths[i]):
                        is_good_length=True
                        rand_lengths.append(a_length)
            confg2 = make_robot_config_from_ee1(config.points[0][0], config.points[0][1],
                                                rand_angles,rand_lengths,ee1_grappled = True)
        else:
            for i in range(spec.num_segments):
                rand_d = random.uniform(-d, d)
                delt_angle = Angle(radians=rand_d)
                an_angle=delt_angle+config.ee2_angles[i]
                rand_angles.append(an_angle)
                is_good_length=False
                while is_good_length==False:
                    rand_d = random.uniform(-d, d)
                    a_length=config.lengths[i]+rand_d*d/1.3*(spec.max_lengths[i]-spec.min_lengths[i])
                    if (a_length<=spec.max_lengths[i] and a_length>=spec.min_lengths[i]):
                        rand_lengths.append(a_length)
                        is_good_length=True
            confg2 = make_robot_config_from_ee2(config.points[-1][0], config.points[-1][1],
                                                rand_angles, rand_lengths,ee2_grappled=True)
        return confg2



    def naive_conf_sampling(config):
        """
        random choose a grapple point and a grapple arm
        """
        rand_angles = []
        rand_lengths = []
        for i in range(spec.num_segments):
            an_angle=Angle(degrees=random.randint(-165,165))#it will be convert to and stored as radius automatically
            rand_angles.append(an_angle)
            temp=random.uniform(spec.min_lengths[i],spec.max_lengths[i])
            rand_lengths.append(round(temp,2))
        if (config.ee1_grappled==True):
            confg2 = make_robot_config_from_ee1(config.points[0][0],config.points[0][1],
                                                rand_angles,rand_lengths,ee1_grappled = True)
        else:
            confg2 = make_robot_config_from_ee2(config.points[-1][0],config.points[-1][1],
                                                rand_angles, rand_lengths,ee2_grappled=True)
        return confg2

    def passage_conf_sampling(config,d=1.5, n=10):
        # d = 1.5  # diameter from obstacle
        # n = 10  # sampling n times in the circle around obstacle point
        isIn_obstacle = False
        while isIn_obstacle == False:
            c1 = naive_conf_sampling(config)
            if vcolli_onlyObstacle_check(c1): isIn_obstacle = True
        # debug_reslist=[]
        # debug_reslist.append(c1)
        isIn_obstacle = False
        count = 0
        while isIn_obstacle == False and count < 10:
            count = count + 1
            c2 = inD_sampling(c1, d)
            if vcolli_onlyObstacle_check(c2): isIn_obstacle = True
        if isIn_obstacle == False: return False
        # debug_reslist.append(c2)
        c_mid = get_mid_c(c1, c2)
        # debug_reslist.append(c_mid)
        if not vcolli_check(c_mid):
            return c_mid  # debug_reslist
        return False

    def obstacle_conf_sampling(config,d=0.2, n=10):
        # d=0.2 #diameter from obstacle
        # n=10 #sampling n times in the circle around obstacle point
        isIn_obstacle = False
        while isIn_obstacle == False:
            c1 = naive_conf_sampling(config)
            if vcolli_onlyObstacle_check(c1): isIn_obstacle = True
        count = 0
        # debug_reslist=[]
        # debug_reslist.append(c1)
        while count < 10:
            count = count + 1
            c2 = inD_sampling(c1, d)
            # debug_reslist.append(c2)
            if not vcolli_check(c2):
                # debug_reslist.append(c2)
                return c2  # debug_reslist
        return False

    def naive_sampling(conf=None):
        """
        random choose a grapple point and a grapple arm
        """
        grapple_at=random.randint(0,len(spec.grapple_points)-1)
        grapple_arm=random.choice([1,2])
        rand_angles = []
        rand_lengths = []
        for i in range(spec.num_segments):
            an_angle=Angle(degrees=random.randint(-165,165))#it will be convert to and stored as radius automatically
            rand_angles.append(an_angle)
            temp=random.uniform(spec.min_lengths[i],spec.max_lengths[i])
            rand_lengths.append(round(temp,2))
        if (grapple_arm==1):
            confg2 = make_robot_config_from_ee1(spec.grapple_points[grapple_at][0],spec.grapple_points[grapple_at][1],
                                                rand_angles,rand_lengths,ee1_grappled = True)
        else:
            confg2 = make_robot_config_from_ee2(spec.grapple_points[grapple_at][0],spec.grapple_points[grapple_at][1],
                                                rand_angles, rand_lengths,ee2_grappled=True)
        return confg2

    def passage_sampling(d = 1.5,n=10):
        # d = 1.5  # diameter from obstacle
        # n = 10  # sampling n times in the circle around obstacle point
        isIn_obstacle = False
        while isIn_obstacle == False:
            c1 = naive_sampling()
            if vcolli_onlyObstacle_check(c1): isIn_obstacle = True
        # debug_reslist=[]
        # debug_reslist.append(c1)
        isIn_obstacle = False
        count = 0
        while isIn_obstacle == False and count<10:
            count = count + 1
            c2 = inD_sampling(c1,d)
            if vcolli_onlyObstacle_check(c2): isIn_obstacle = True
        if isIn_obstacle==False: return False
        # debug_reslist.append(c2)
        c_mid = get_mid_c(c1, c2)
        # debug_reslist.append(c_mid)
        if not vcolli_check(c_mid):
            return c_mid #debug_reslist
        return False

    def obstacle_sampling(d=0.2,n=10):
        """
        :return: return a config near obstacle or False if no such point
        """
        # d=0.2 #diameter from obstacle
        # n=10 #sampling n times in the circle around obstacle point
        isIn_obstacle=False
        while isIn_obstacle==False:
            c1=naive_sampling()
            if vcolli_onlyObstacle_check(c1):isIn_obstacle=True
        count=0
        #debug_reslist=[]
        #debug_reslist.append(c1)
        while count<10:
            count=count+1
            c2=inD_sampling(c1,d)
            # debug_reslist.append(c2)
            if not vcolli_check(c2):
                #debug_reslist.append(c2)
                return c2 #debug_reslist
        return False

    def one_point_sampling(name,config=None,c=None,d=None,n=None):
        count=0
        is_good_sample=False
        while(is_good_sample==False and count<50):
            count=count+1
            if name=="naive":res_config=naive_sampling()
            elif name == "passage": res_config = passage_sampling()
            elif name == "obstacle": res_config = obstacle_sampling()
            elif name =="passage_conf":res_config=passage_conf_sampling(config)
            else:return False
            if res_config is not False and not vcolli_check(res_config):
                return res_config
        return False



    def combine_sampling_1(n):
        for i in range(math.ceil(n/3)):
            v = one_point_sampling("naive")
            if v is not False:V.append(GraphNode(spec, v))
            v = one_point_sampling("passage")
            if v is not False: V.append(GraphNode(spec, v))
            v = one_point_sampling("obstacle")
            if v is not False: V.append(GraphNode(spec, v))





    """====================sampling========================"""
    """======================collision====================="""

    def interpolation(c1,c2):
        """
        return a list of config
        choose the abs max one, divided into 0.001 and n fractions. Then for all the other dim also get n fractions
        :param c1:
        :param c2:
        :return:
        """
        max_ee1_delta = 0
        max_ee2_delta = 0
        ee1_delta=[] #total movement of each arm, a list of delta
        ee2_delta=[]
        length_delta=[]
        for i in range(spec.num_segments):
            ee1_delta.append(c2.ee1_angles[i] - c1.ee1_angles[i])
            if abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians()) > max_ee1_delta:
                max_ee1_delta = abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians())

            ee2_delta.append(c2.ee2_angles[i] - c1.ee2_angles[i])
            if abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians()) > max_ee2_delta:
                max_ee2_delta = abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians())

        # measure leniently - allow compliance from EE1 or EE2
        max_delta = min(max_ee1_delta, max_ee2_delta)

        for i in range(spec.num_segments):
            length_delta.append(c2.lengths[i] - c1.lengths[i])
            if abs(c2.lengths[i] - c1.lengths[i]) > max_delta:
                max_delta = abs(c2.lengths[i] - c1.lengths[i])

        if max_delta <= spec.PRIMITIVE_STEP + spec.TOLERANCE:
            return [c1,c2]
        else:
            fraction_num=math.ceil(max_delta/spec.PRIMITIVE_STEP)
            config_list=[]
            config_list.append(c1)
            if (c1.ee1_grappled == True and c2.ee1_grappled==True):
                for i in range(fraction_num):
                    rand_angles=[]
                    rand_lengths=[]
                    for j in range(spec.num_segments):
                        an_angle = ee1_delta[j]* (i/fraction_num)
                        an_angle=an_angle+c1.ee1_angles[j]
                        rand_angles.append(an_angle)
                        rand_lengths.append(c1.lengths[j]+length_delta[j]* (i/fraction_num))
                    config = make_robot_config_from_ee1(c1.points[0][0], c1.points[0][1], rand_angles, rand_lengths, ee1_grappled=True)
                    config_list.append(config)
            elif(c1.ee2_grappled == True and c2.ee2_grappled==True):
                for i in range(fraction_num):
                    rand_angles = []
                    rand_lengths = []
                    for j in range(spec.num_segments):
                        an_angle = ee2_delta[j] * (i/fraction_num)
                        an_angle = an_angle + c1.ee2_angles[j]
                        rand_angles.append(an_angle)
                        rand_lengths.append(c1.lengths[j]+length_delta[j] * (i/fraction_num))
                    config = make_robot_config_from_ee2(c1.points[-1][0], c1.points[-1][1], rand_angles, rand_lengths, ee2_grappled=True)
                    config_list.append(config)
            elif(c1.ee1_grappled == True and c2.ee1_grappled==False):
                print("c1 to c2 change the grapple point, impossible to interpolation")
            elif (c1.ee2_grappled == True and c2.ee2_grappled == False):
                print("c1 to c2 change the grapple point, impossible to interpolation")
            config_list.append(c2)
            return config_list

    def vcolli_check(config):
        """
        :param config:
        :return: True if is collide
        """
        global check_v
        check_v+=1
        if not (test_environment_bounds(config)): return True
        if not (test_obstacle_collision(config, spec, spec.obstacles)): return True
        if not (test_self_collision(config, spec)):return True
        if not (test_grapple_point_constraint(config, spec)): return True
        if not (test_length_constraints(config, spec)): return True
        if not (test_angle_constraints(config, spec)): return True
        return False

    def vcolli_onlyObstacle_check(config):
        """
        :param config:
        :return: True if only Obstacle is collide
        """
        global check_v
        check_v+=1
        if not (test_environment_bounds(config)): return False
        if not (test_self_collision(config, spec)): return False
        if not (test_grapple_point_constraint(config, spec)): return False
        if not (test_length_constraints(config, spec)): return False
        if not (test_angle_constraints(config, spec)): return False
        if not (test_obstacle_collision(config, spec, spec.obstacles)): return True
        return False

    def ecolli_check2(c1,c2):
        """
        use interporlation to create a list, and check each node by vcolli_check!!!
        :param c1:
        :param c2:
        :return:
        """
        global check_e
        check_e+=1
        dist=0.05
        if distance(c1, c2) <= dist: return False
        q=[]
        q.append((c1,c2))
        count=0
        while q:

            temp=q.pop(0)
            p1=temp[0]
            p2=temp[1]

            """p_mid create"""
            #calculate delta
            ee1_delta = []  # total movement of each arm, a list of delta
            ee2_delta = []
            length_delta = []
            for i in range(spec.num_segments):
                ee1_delta.append(p2.ee1_angles[i] - p1.ee1_angles[i])
                ee2_delta.append(p2.ee2_angles[i] - p1.ee2_angles[i])
                length_delta.append(p2.lengths[i] - p1.lengths[i])
            #create p_mid
            if (p1.ee1_grappled == True and p2.ee1_grappled==True):
                rand_angles=[]
                rand_lengths=[]
                for j in range(spec.num_segments):
                    an_angle = ee1_delta[j]/2
                    an_angle=an_angle+p1.ee1_angles[j]
                    rand_angles.append(an_angle)
                    rand_lengths.append(p1.lengths[j]+length_delta[j]/2)
                p_mid= make_robot_config_from_ee1(p1.points[0][0], p1.points[0][1], rand_angles, rand_lengths, ee1_grappled=True)
            elif(p1.ee2_grappled == True and p2.ee2_grappled==True):
                rand_angles = []
                rand_lengths = []
                for j in range(spec.num_segments):
                    an_angle = ee2_delta[j] / 2
                    an_angle = an_angle + p1.ee2_angles[j]
                    rand_angles.append(an_angle)
                    rand_lengths.append(p1.lengths[j]+length_delta[j] / 2)
                p_mid= make_robot_config_from_ee2(p1.points[-1][0], p1.points[-1][1], rand_angles, rand_lengths, ee2_grappled=True)

            if vcolli_check(p_mid):return True
            if distance(p1,p_mid)> dist:
                q.append((p1,p_mid))
            else:count=count+1
            if distance(p_mid,p2) > dist:
                q.append((p_mid,p2))
            else:count=count+1
        return False

    def ecolli_check1(c1,c2):
        """
        use interporlation to create a list, and check each node by vcolli_check!!!
        :param c1:
        :param c2:
        :return:
        """
        global check_e
        check_e+=1
        config_list=interpolation(c1,c2)
        for i in range(len(config_list)):
            if vcolli_check(config_list[i]): return True
        return False

    def distance(c1,c2):
        """
        if c1 only grab arm1, c2 only grab arm 2, return very large!!
        not allow change grab arm in one edge(move).
        :param c1:
        :param c2:
        :return:
        """
        if c1.ee1_grappled != c2.ee1_grappled: return 1000
        if c1.ee2_grappled != c2.ee2_grappled: return 1000
        if c1.ee1_grappled==True: p1=c1.points[0]; p2=c2.points[0]
        else: p1=c1.points[-1]; p2=c2.points[-1]
        if p1 != p2: return 1000
        max_ee1_delta = 0
        max_ee2_delta = 0
        ee1_delta = []  # total movement of each arm, a list of delta
        ee2_delta = []
        length_delta = []
        for i in range(spec.num_segments):
            ee1_delta.append(abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians()))
            if abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians()) > max_ee1_delta:
                max_ee1_delta = abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians())

            ee2_delta.append(abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians()))
            if abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians()) > max_ee2_delta:
                max_ee2_delta = abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians())

        # measure leniently - allow compliance from EE1 or EE2
        max_delta = min(max_ee1_delta, max_ee2_delta)

        for i in range(spec.num_segments):
            length_delta.append(abs(c2.lengths[i] - c1.lengths[i]))
            if abs(c2.lengths[i] - c1.lengths[i]) > max_delta:
                max_delta = abs(c2.lengths[i] - c1.lengths[i])
        return max_delta


    """=========================collision============================="""
    """=========================connection============================"""

    def connect_edges():
        for i in range(len(V) - 1):
            for l in range(len(V) - i - 1):
                j = l + i + 1
                if distance(V[i].config, V[j].config) < 1:
                    if not ecolli_check2(V[i].config, V[j].config):
                        GraphNode.add_connection(V[i], V[j])
                        E.append((V[i], V[j]))

    def connect_OnlyNewVetex(start_num):
        #only new V are linking edge
        for i in range(start_num,len(V)):
            for j in range(len(V)):
                if i==j:continue
                if distance(V[i].config, V[j].config) < 1:
                    if not ecolli_check2(V[i].config, V[j].config):
                        GraphNode.add_connection(V[i], V[j])
                        E.append((V[i], V[j]))


    """==================above methods================================"""

    """==================below main==================================="""
    """Prinpicle of Robot Motion p 204, changed a bit as here is an implicit Graph"""
    find_path=False
    n=20
    k=0.1
    V=[]
    E=[]
    V.append(init_node)
    V.append(goal_node)
    disjoint_sets={}
    count=0
    global check_e
    global check_e2
    check_e=0
    check_e2=0
    global check_v
    check_v=0
    start=time.time()


    """====================sampling========================"""

    print("main, V: ",len(V)," E:",len(E))
    while find_path==False and count<1:
        old_V=len(V)
        count=count+1
        if(count%10==0):print("current nodes: ",count*n)
        combine_sampling_1(n)

        """====================sampling========================"""

        # connect_OnlyNewVetex(old_V)
        print(" count(# of while loop): ", count * n)
        print("checked edges(not including find bridges): ", check_e, " checked vertex: ", check_v, " valid edges: ",
              len(E), " valid vetex(in V): ", len(V))

        # steps=find_graph_path(spec,init_node)
        # if steps is not None: find_path=True

    """debug"""
    end=time.time()
    print("time: ",end-start," count(# of while loop): ",count*n," path length: ",len(steps))
    print("checked edges(not including find bridges): ",check_e," checked vertex: ", check_v," valid edges: ",len(E)," valid vetex(in V): ",len(V))


    for i in range(len(V) - 1):
        steps.append(V[i].config)
    """add all interpolation points"""
    # temp=[]
    # for i in range(len(steps)-1):
    #     x=interpolation(steps[i],steps[i+1])
    #     x.pop()
    #     temp=temp+x
    # temp.append(steps[-1])
    # steps=temp

    # if len(arglist) > 1:
    if 1:
        write_robot_config_list_to_file(output_file, steps)

    #
    # You may uncomment this line to launch visualiser once a solution has been found. This may be useful for debugging.
    # *** Make sure this line is commented out when you submit to Gradescope ***
    #
    v=Visualiser(spec, steps)


if __name__ == '__main__':
    # main(sys.argv[1:])
    main()



