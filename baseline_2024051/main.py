import json

from util import *
# from myalgorithm import algorithm
from myalgorithm import algorithm
import logging
import glob
import subprocess
import multiprocessing

problem_file = '../alg_test_problems_20240429/TEST_K200_2.json'
timelimit = 10
MOE = ['total_cost', 'avg_cost', 'num_drivers', 'total_dist']


def main():
    folder_path = "../alg_test_problems_20240429"
    all_data = {}
    all_solution = {}
    for filename in glob.glob(f"{folder_path}/*.json"):
        with open(filename, "r") as f:
            data = json.load(f)
            key_name = str(filename.split("\\")[1].split(".json")[0])
            all_data[key_name] = data

    for file_name, input_data in all_data.items():
        print(f'file name: {file_name}')
        # Now you have a list `all_data` containing the data from each JSON file
        checked_solution = solve(input_data)
        all_solution[file_name] = checked_solution
        print(checked_solution)

    with open('all_solution.json', 'r') as f:
        prev_solutions = json.load(f)
        for key, prev_solution in prev_solutions.items():
            print(f'Compare Plan: {key}')
            curr_solution = all_solution[key]

            for moe in MOE:

                prev_cost = int(prev_solution[moe])
                curr_cost = int(curr_solution[moe])

                if prev_cost == curr_cost:
                    print(f'    {moe} Cost is the same')
                elif curr_cost < prev_cost:
                    print(f'    {moe} Cost Decreased! prev: {prev_cost}, curr: {curr_cost}')
                else:
                    print(f'    {moe} Cost Increased! prev: {prev_cost}, curr: {curr_cost}')


    # write_solutions(all_solution)



def write_solutions(all_solution):
    with open('all_solution.json', 'w') as output_file:
        json.dump(all_solution, output_file, indent=4)


def solve(prob):
    K = prob['K']
    ALL_ORDERS = [Order(order_info) for order_info in prob['ORDERS']]
    ALL_RIDERS = [Rider(rider_info) for rider_info in prob['RIDERS']]
    DIST = np.array(prob['DIST'])
    for r in ALL_RIDERS:
        r.T = np.round(DIST / r.speed + r.service_time)
    alg_start_time = time.time()
    exception = None
    solution = None
    try:
        # Run algorithm!
        solution = algorithm(K, ALL_ORDERS, ALL_RIDERS, DIST, timelimit)
    except Exception as e:
        exception = f'{e}'
        logging.exception(exception, e)
        # logging.error(exception)
    alg_end_time = time.time()

    checked_solution = solution_check(K, ALL_ORDERS, ALL_RIDERS, DIST, solution)
    checked_solution['time'] = alg_end_time - alg_start_time
    checked_solution['timelimit_exception'] = (
                                                      alg_end_time - alg_start_time) > timelimit + 1  # allowing additional 1 second!
    checked_solution['exception'] = exception
    checked_solution['prob_name'] = prob['name']
    checked_solution['prob_file'] = problem_file
    return checked_solution


if __name__ == "__main__":
    main()
