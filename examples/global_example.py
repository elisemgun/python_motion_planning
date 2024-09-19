"""
@file: global_example.py
@breif: global planner application examples
@author: Winter
@update: 2023.3.2
"""
import os
import sys
import random
import csv
import time
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from python_motion_planning.utils import Grid, Map, SearchFactory


if __name__ == '__main__':
    '''
    path searcher constructor
    '''
    search_factory = SearchFactory()

    '''
    graph search
    '''
    # build environment
    env = Grid(51, 31)

    # --------------------------------------------------------
    # Create a grid environment and run a series of tests
    # --------------------------------------------------------

    algorithms = ["a_star", "dijkstra", "theta_star", "jps", "lazy_theta_star"]
    results = []

    def get_random_point(env):
        env_width, env_height = env.x_range, env.y_range
        point = (random.randint(0, env_width - 1), random.randint(0, env_height - 1))
        
        # Ensure the point is not an obstacle
        while point in env.obstacles:
            point = (random.randint(0, env_width - 1), random.randint(0, env_height - 1))
        
        return point
    
    # Open the CSV file once at the beginning
    with open('planner_results.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Algorithm", "Time (seconds)", "Cost", "Path", "Cost length?", "Start Point", "Goal Point"])

        for i in range(1000):
            start = get_random_point(env)
            goal = get_random_point(env)

            while start == goal:
                goal = get_random_point(env)

            for j in range(10):
                for alg in algorithms:
                    planner = search_factory(alg, start=start, goal=goal, env=env)

                    start_time = time.perf_counter()
                    cost, path, expanded = planner.plan()

                    # cost is all nodes visited
                    # path is the path from start to goal

                    end_time = time.perf_counter()
                    elapsed_time = end_time - start_time

                    length = len(path)
                    expanded = len(expanded)
                    
                    results.append([alg, elapsed_time, cost, length, expanded, start, goal])

                # Write results to the CSV every second iteration (j % 2 == 1)
                if j % 2 == 1:
                    writer.writerows(results)
                    file.flush()  # Ensure that the data is written to disk
                    results = []  # Clear results after writing to avoid duplicate entries

    print("Results saved to planner_results.csv")


    # # --------------------------------------------------------
            
    # create planner

    # planner = search_factory("a_star", start=start, goal=goal, env=env)
    # planner = search_factory("dijkstra", start=start, goal=goal, env=env)
    # planner = search_factory("theta_star", start=start, goal=goal, env=env)
    # planner = search_factory("lazy_theta_star", start=start, goal=goal, env=env)
    # planner = search_factory("jps", start=start, goal=goal, env=env)

    # planner = search_factory("gbfs", start=start, goal=goal, env=env)
    # planner = search_factory("s_theta_star", start=start, goal=goal, env=env)
    # planner = search_factory("d_star", start=start, goal=goal, env=env)
    # planner = search_factory("lpa_star", start=start, goal=goal, env=env)
    # planner = search_factory("d_star_lite", start=start, goal=goal, env=env)
    # planner = search_factory("voronoi", start=start, goal=goal, env=env, n_knn=4,
    #                             max_edge_len=10.0, inflation_r=1.0)

    # animation
    # planner.run()

    # ========================================================

    '''
    sample search
    '''
    # # build environment
    # start = (18, 8)
    # goal = (37, 18)
    # env = Map(51, 31)

    # # creat planner
    # planner = search_factory("rrt", start=start, goal=goal, env=env)
    # planner = search_factory("rrt_connect", start=start, goal=goal, env=env)
    # planner = search_factory("rrt_star", start=start, goal=goal, env=env)
    # planner = search_factory("informed_rrt", start=start, goal=goal, env=env)

    # # animation
    # planner.run()

    # ========================================================

    '''
    evolutionary search
    '''
    # planner = search_factory("aco", start=start, goal=goal, env=env)
    # planner = search_factory("pso", start=start, goal=goal, env=env)
    # planner.run()

