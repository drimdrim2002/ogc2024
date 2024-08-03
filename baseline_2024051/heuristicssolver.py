import os
import subprocess
import json

MAX_ARG_SIZE = 30000


def get_jar_file_name():
    jar_path = os.path.realpath(__file__)
    jar_path = jar_path[0: jar_path.rfind("\\")]
    jar_path += "\OgcHeuristics.jar"
    return jar_path


def get_duration(dist_mat, orgin_indexes, destination_indexes):
    duration_all_edges = {}
    for origin_index in range(orgin_indexes[0], orgin_indexes[1]):
        duration_all_edges[origin_index] = {}
        for destination_index in range(destination_indexes[0], destination_indexes[1]):
            duration_edge = dist_mat[origin_index][destination_index] if origin_index != destination_index else 0
            duration_all_edges[origin_index][destination_index] = str(duration_edge)
    return duration_all_edges
def get_distance(dist_mat, orgin_indexes, destination_indexes):
    distance_all_edges = {}
    for origin_index in range(orgin_indexes[0], orgin_indexes[1]):
        distance_all_edges[origin_index] = {}
        for destination_index in range(destination_indexes[0], destination_indexes[1]):
            distance_edge = dist_mat[origin_index][destination_index] if origin_index != destination_index else 0
            distance_all_edges[origin_index][destination_index] = str(distance_edge)
    return distance_all_edges


def get_original_input(K, all_orders, all_riders, dist_mat):
    order_dict = {}
    for order in all_orders:
        order_dict[order.id] = {}
        order_dict[order.id]['order_time'] = order.order_time
        order_dict[order.id]['cook_time'] = order.cook_time
        order_dict[order.id]['volume'] = order.volume
        order_dict[order.id]['deadline'] = order.deadline
    rider_dict = {}
    for rider in all_riders:
        rider_dict[rider.type] = {}
        rider_dict[rider.type]['type'] = rider.type
        rider_dict[rider.type]['speed'] = rider.speed
        rider_dict[rider.type]['capa'] = rider.capa
        rider_dict[rider.type]['var_cost'] = rider.var_cost
        rider_dict[rider.type]['fixed_cost'] = rider.fixed_cost
        rider_dict[rider.type]['service_time'] = rider.service_time
        rider_dict[rider.type]['available_number'] = rider.available_number
        rider_dict[rider.type]['duration_shops'] = get_duration(rider.T, [0, K], [0, K])
        rider_dict[rider.type]['duration_shops_to_dlvrys'] = get_duration(rider.T, [0, K], [K, 2 * K])
        rider_dict[rider.type]['duration_dlvrys'] = get_duration(rider.T, [K, 2 * K], [K, 2 * K])
    dist_shops = get_distance(dist_mat, [0, K], [0, K])
    dist_shops_to_dlvrys = get_distance(dist_mat, [0, K], [K, 2 * K])
    dist_dlvrys = get_distance(dist_mat, [K, 2 * K], [K, 2 * K])


    original_data = {'orders': json.dumps(order_dict), 'riders': json.dumps(rider_dict),
                     'dist_shops': json.dumps(dist_shops), 'dist_shops_to_dlvrys': json.dumps(dist_shops_to_dlvrys),
                     'dist_dlvrys': json.dumps(dist_dlvrys)}
    original_data_str = str(original_data)
    return original_data_str


def get_initial_solution(K, all_orders, all_riders, dist_mat, vehicle_type_by_index):
    heuristics_result = call_heuristics_result(K, all_orders, all_riders, dist_mat)
    initial_solution = []

    print(heuristics_result['cost'])
    for vehicle_id in vehicle_type_by_index.keys():
        route_seq = []
        vehicle_id_str = str(vehicle_id)
        if vehicle_id_str in heuristics_result:
            shop_seq_arr = heuristics_result[vehicle_id_str]['shopSeq']
            dlvry_seq_arr = heuristics_result[vehicle_id_str]['deliverySeq']
            for shop_seq in shop_seq_arr:
                route_seq.append((int(shop_seq)) + 1)
            for dlvry_seq in dlvry_seq_arr:
                route_seq.append((int(dlvry_seq + K)) + 1)

            initial_solution.append(route_seq)
    # for i in range(1, 101) :
    #     route = []
    #     route.append(i)
    #     route.append(i + K)
    #     initial_solution.append(route)
    return initial_solution




def set_system_environment(original_data_str):
    substr_start = 0
    substr_end = min(substr_start + MAX_ARG_SIZE, len(original_data_str))
    args = []
    arg_index = 0
    while True:
        environ_param = "alpha_input" + "_" + str(arg_index)
        args.append(environ_param)
        os.environ[environ_param] = original_data_str[substr_start: substr_end]
        arg_index += 1
        if substr_end >= len(original_data_str):
            break
        else:
            substr_start = substr_end
            substr_end = min(substr_start + MAX_ARG_SIZE, len(original_data_str))
    return args


def call_heuristics_result(K, all_orders, all_riders, dist_mat):
    jar_path = get_jar_file_name()
    original_data_str = get_original_input(K, all_orders, all_riders, dist_mat)
    print(original_data_str)
    args = set_system_environment(original_data_str)
    heuristics_result = ''
    # Build the command to execute the JAR with arguments
    command = ["java", "-jar", jar_path]  # Convert numbers to strings
    try:

        # Execute the JAR and capture the output (may include errors)
        result = subprocess.run(command + args, capture_output=True, encoding='utf-8')

        # Extract the sum result from the output (assuming the JAR returns it on a single line)
        heuristics_result = json.loads(result.stdout.strip())
    except Exception as error:
        print(f"Error executing JAR: {error}")
    return heuristics_result
