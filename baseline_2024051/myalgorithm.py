from itertools import permutations

import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math
from datetime import datetime

BIG_PENALTY_VALUE = 99999999


def algorithm(K, all_orders, all_riders, dist_mat, timelimit=60):
    # print(f'K : {K}')

    data = make_input_data(K, dist_mat, all_orders, all_riders)

    # Create the routing index manager.
    # [START index_manager]
    manager = pywrapcp.RoutingIndexManager(
        len(data['distance_matrix']), data['num_vehicles'], data['depot'])
    # [END index_manager]

    routing = pywrapcp.RoutingModel(manager)

    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    rider_cost_info = {}
    rider_cost_info['CAR'] = {}
    rider_cost_info['BIKE'] = {}
    rider_cost_info['WALK'] = {}

    for rider in all_riders:
        fixed_cost = rider.fixed_cost
        var_cost = rider.var_cost
        if rider.type == 'CAR':
            rider_cost_info['CAR']['fixed_cost'] = fixed_cost
            rider_cost_info['CAR']['var_cost'] = var_cost
        elif rider.type == 'BIKE':
            rider_cost_info['BIKE']['fixed_cost'] = fixed_cost
            rider_cost_info['BIKE']['var_cost'] = var_cost
        else:
            rider_cost_info['WALK']['fixed_cost'] = fixed_cost
            rider_cost_info['WALK']['var_cost'] = var_cost

    # Create and register a transit callback.
    def time_callback_car(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        duration = data["time_matrix_car"][from_node][to_node]
        return duration

    transit_callback_index_car = routing.RegisterTransitCallback(time_callback_car)

    def time_callback_bike(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["time_matrix_bike"][from_node][to_node]

    transit_callback_index_bike = routing.RegisterTransitCallback(time_callback_bike)

    def time_callback_walk(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["time_matrix_walk"][from_node][to_node]

    transit_callback_index_walk = routing.RegisterTransitCallback(time_callback_walk)

    def cost_callback_car(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        distance = data["distance_matrix"][from_node][to_node]
        if distance == BIG_PENALTY_VALUE:
            return BIG_PENALTY_VALUE

        duration = data["time_matrix_car"][from_node][to_node]
        if duration == BIG_PENALTY_VALUE:
            return BIG_PENALTY_VALUE
        car_var_cost = rider_cost_info['CAR']['var_cost']

        car_fixed_cost = rider_cost_info['CAR']['fixed_cost'] if from_node == 0 else 0
        return int(car_fixed_cost) + int((distance / 100) * car_var_cost)

    # cost_callback_car.SetGlobalSpanCostCoefficient(100)
    cost_callback_index_car = routing.RegisterTransitCallback(cost_callback_car)

    def cost_callback_bike(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        distance = data["distance_matrix"][from_node][to_node]
        if distance == BIG_PENALTY_VALUE:
            return BIG_PENALTY_VALUE

        duration = data["time_matrix_bike"][from_node][to_node]
        if duration == BIG_PENALTY_VALUE:
            return BIG_PENALTY_VALUE
        bike_fixed_cost = rider_cost_info['BIKE']['fixed_cost'] if from_node == 0 else 0
        bike_var_cost = rider_cost_info['BIKE']['var_cost']
        return int(bike_fixed_cost) + int((distance / 100) * bike_var_cost)

    # cost_callback_bike.SetGlobalSpanCostCoefficient(100)

    cost_callback_index_bike = routing.RegisterTransitCallback(cost_callback_bike)

    def cost_callback_walk(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        distance = data["distance_matrix"][from_node][to_node]
        if distance == BIG_PENALTY_VALUE:
            return BIG_PENALTY_VALUE
        duration = data["time_matrix_walk"][from_node][to_node]
        if duration == BIG_PENALTY_VALUE:
            return BIG_PENALTY_VALUE
        walk_fixed_cost = rider_cost_info['WALK']['fixed_cost'] if from_node == 0 else 0

        walk_var_cost = rider_cost_info['WALK']['var_cost']
        return int(walk_fixed_cost) + int((distance / 100) * walk_var_cost)

    # cost_callback_walk.SetGlobalSpanCostCoefficient(100)

    cost_callback_index_walk = routing.RegisterTransitCallback(cost_callback_walk)

    transit_callback_arr = []
    for vehicle_index in range(len(data["vehicle_type_by_index"])):
        vehicle_type = data["vehicle_type_by_index"][vehicle_index]
        if vehicle_type == 'CAR':
            transit_callback_arr.append(transit_callback_index_car)
            routing.SetArcCostEvaluatorOfVehicle(cost_callback_index_car, vehicle_index)
        elif vehicle_type == 'BIKE':
            transit_callback_arr.append(transit_callback_index_bike)
            routing.SetArcCostEvaluatorOfVehicle(cost_callback_index_bike, vehicle_index)
        else:
            transit_callback_arr.append(transit_callback_index_walk)
            routing.SetArcCostEvaluatorOfVehicle(cost_callback_index_walk, vehicle_index)

    # routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index_car)

    # Add Time Windows constraint.
    time = "Time"
    routing.AddDimensionWithVehicleTransits(
        transit_callback_arr,
        0,  # allow waiting time
        BIG_PENALTY_VALUE - 1000,  # maximum time per vehicle todo planning horizon을 확인해야 함
        False,  # Don't force start cumul to zero.
        time,
    )

    time_dimension = routing.GetDimensionOrDie(time)

    # Define Transportation Requests.
    for request in data['pickups_deliveries']:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        routing.solver().Add(time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index))

    # Add time window constraints for each location except depot.
    for location_idx, time in enumerate(data["time_windows"]):
        if location_idx == data["depot"]:
            continue

        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time[0], time[1])

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.LOCAL_CHEAPEST_INSERTION)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 50

    print(f'solve start time: {datetime.now().strftime("%H:%M:%S")}')

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    # print("Solver status: ", assignment.status())
    # Print solution on console.
    if assignment:
        print_solution_simple(data, manager, routing, assignment, all_riders, K)
        solution_bundle_arr = make_solution_bundle(data, manager, routing, assignment)

        solution_bundle_by_type = {}
        solution_bundle_by_type['CAR'] = []
        solution_bundle_by_type['BIKE'] = []
        solution_bundle_by_type['WALK'] = []
        for solution_bundle in solution_bundle_arr:
            vehicle_type = solution_bundle[0]
            shop_seq = solution_bundle[1]
            solution_bundle_by_type[vehicle_type].append(shop_seq)

        for vehicle_type in solution_bundle_by_type.keys():
            print(vehicle_type)
            dlvry_seq_by_type = solution_bundle_by_type[vehicle_type]
            print(dlvry_seq_by_type)

        return solution_bundle_arr
    else:
        print("No assignment")


def make_input_data(K, dist_mat, all_orders, all_riders):
    data = {}

    data["depot"] = 0  # dummy depot

    data["distance_matrix"] = make_distance_matrix(K, dist_mat)

    data["pickups_deliveries"] = make_pickup_delivery(K)

    data["demands"] = make_demand(all_orders)

    data["vehicle_type_by_index"] = {}
    vehicle_capacity_arr = []
    num_vehicles = 0

    vehicle_index = 0

    for rider in all_riders:
        if rider.type == 'CAR':
            car_rider = rider
        elif rider.type == 'BIKE':
            bike_rider = rider
        else:
            walk_rider = rider

    ordered_riders = []
    ordered_riders.append(car_rider)
    ordered_riders.append(bike_rider)
    ordered_riders.append(walk_rider)

    for rider in ordered_riders:
        num_vehicles += rider.available_number
        for _ in range(rider.available_number):
            vehicle_capacity_arr.append(rider.capa)
            data["vehicle_type_by_index"][vehicle_index] = rider.type
            vehicle_index += 1

        time_matrix = np.zeros((2 * K + 1, 2 * K + 1))
        for row_index in range(0, 2 * K + 1):
            for column_index in range(0, 2 * K + 1):
                distance = data["distance_matrix"][row_index][column_index]
                if distance == 0:
                    time_matrix[row_index][column_index] = int(math.ceil(distance / rider.speed + rider.service_time))
                elif distance == BIG_PENALTY_VALUE:
                    time_matrix[row_index][column_index] = int(BIG_PENALTY_VALUE)
                else:
                    time_matrix[row_index][column_index] = int(math.ceil(distance / rider.speed + rider.service_time))

        if rider.type == 'CAR':
            data["time_matrix_car"] = time_matrix
            data["time_matrix_car"] = data["time_matrix_car"].astype(int).tolist()
        elif rider.type == 'BIKE':
            data["time_matrix_bike"] = time_matrix
            data["time_matrix_bike"] = data["time_matrix_bike"].astype(int).tolist()

        else:
            data["time_matrix_walk"] = time_matrix
            data["time_matrix_walk"] = data["time_matrix_walk"].astype(int).tolist()

    data["time_windows"] = make_time_window(all_orders, data["time_matrix_car"])

    data["num_vehicles"] = num_vehicles
    data["vehicle_capacities"] = vehicle_capacity_arr

    apply_time_penalty(K, all_orders, data["time_matrix_car"])
    apply_time_penalty(K, all_orders, data["time_matrix_bike"])
    apply_time_penalty(K, all_orders, data["time_matrix_walk"])

    return data


def combinations(k):
    if k < 2:
        raise ValueError("k는 2보다 커야 합니다.")
    combs = []
    for i in range(1, k + 1):
        for j in range(i + 1, k + 1):
            combs.append((i, j))
    return combs


def apply_time_penalty(K, all_orders, _time_matrix):
    for from_order in all_orders:
        from_shop_index = from_order.id + 1
        from_cust_index = from_shop_index + K

        for to_order in all_orders:
            to_shop_index = to_order.id + 1
            to_cust_index = to_shop_index + K

            if from_shop_index == to_shop_index:
                continue

            pickup_time = from_order.ready_time
            pickup_time += _time_matrix[from_shop_index][to_shop_index]
            pickup_time = max(pickup_time, to_order.ready_time)

            cust_arr = []
            cust_arr.append(from_cust_index)
            cust_arr.append(to_cust_index)

            fail_count = 0
            for cust_pem_seq in permutations(cust_arr):
                first_customer_index = cust_pem_seq[0]
                second_customer_index = cust_pem_seq[1]
                if from_cust_index == first_customer_index:
                    first_order = from_order
                    second_order = to_order
                else:
                    first_order = to_order
                    second_order = from_order

                dlvry_time = pickup_time + _time_matrix[to_shop_index][first_customer_index]
                if dlvry_time > first_order.deadline:
                    if first_customer_index != to_shop_index + K:
                        _time_matrix[to_shop_index][first_customer_index] = BIG_PENALTY_VALUE
                    # _time_matrix[to_shop_index][first_customer_index] = BIG_PENALTY_VALUE
                    # _time_matrix[first_customer_index][second_customer_index] = BIG_PENALTY_VALUE
                    fail_count += 1
                    continue

                dlvry_time += _time_matrix[first_customer_index][second_customer_index]
                if dlvry_time > second_order.deadline:
                    if first_customer_index != to_shop_index + K:
                        _time_matrix[to_shop_index][first_customer_index] = BIG_PENALTY_VALUE
                    # _time_matrix[to_shop_index][first_customer_index] = BIG_PENALTY_VALUE
                    # _time_matrix[first_customer_index][second_customer_index] = BIG_PENALTY_VALUE
                    fail_count += 1

            if fail_count >= 2:
                _time_matrix[from_shop_index][to_shop_index] = BIG_PENALTY_VALUE

    combination_results = combinations(K)
    for combination_result in combination_results:
        first_shop_index = combination_result[0]
        second_shop_index = combination_result[1]
        if (_time_matrix[first_shop_index][second_shop_index] == BIG_PENALTY_VALUE
                and _time_matrix[second_shop_index][first_shop_index] == BIG_PENALTY_VALUE):
            first_customer_index = first_shop_index + K
            second_customer_index = second_shop_index + K

            _time_matrix[first_shop_index][second_customer_index] = BIG_PENALTY_VALUE
            _time_matrix[second_shop_index][first_customer_index] = BIG_PENALTY_VALUE
            _time_matrix[first_customer_index][second_customer_index] = BIG_PENALTY_VALUE
            _time_matrix[second_customer_index][first_customer_index] = BIG_PENALTY_VALUE


def make_demand(all_orders):
    demand_array = [0]
    for order in all_orders:
        demand_array.append(int(order.volume))
    for order in all_orders:
        demand_array.append(int(order.volume * -1))
    return demand_array


def make_distance_matrix(K, dist_mat):
    new_dist_matrix = np.zeros((2 * K + 1, 2 * K + 1))
    for row_index in range(2 * K):
        for column_index in range(2 * K):
            new_dist_matrix[row_index + 1][column_index + 1] = dist_mat[row_index][column_index]

    # set long distance from depot to customer
    for customer_index in range(K + 1, 2 * K + 1):
        new_dist_matrix[0][customer_index] = BIG_PENALTY_VALUE

    # set long distance from customer to shop
    for customer_index in range(K + 1, 2 * K + 1):
        for shop_index in range(1, K + 1):
            new_dist_matrix[customer_index][shop_index] = BIG_PENALTY_VALUE

    # set long distance from shop to customer without pickup
    for sho_index in range(1, K + 1):
        for customer_index in range(K + 1, 2 * K + 1):
            if sho_index + K != customer_index:
                new_dist_matrix[customer_index][shop_index] = BIG_PENALTY_VALUE

    tolist = new_dist_matrix.astype(int).tolist()
    return tolist


def make_pickup_delivery(K):
    np_array = np.zeros((K, 2))
    for order_index in range(K):
        np_array[order_index][0] = int(order_index + 1)
        np_array[order_index][1] = int(order_index + 1 + K)
    return np_array.astype(int).tolist()


def make_time_window(all_orders, _time_matrix):
    deatline_arr = []
    for order in all_orders:
        deatline_arr.append(order.deadline)
    time_window_arr = [(0, max(deatline_arr) + 1)]

    max_shop_dep_dict = {}
    min_cust_arr_dict = {}
    for order in all_orders:
        from_time_matrix_index = order.id + 1
        to_time_matrix_index = from_time_matrix_index + len(all_orders)
        duration = _time_matrix[from_time_matrix_index][to_time_matrix_index]
        max_shop_dep_dict[order.id] = int(order.deadline - duration)
        min_cust_arr_dict[order.id] = int(order.ready_time + duration)
    for order in all_orders:
        max_shop_dep = max_shop_dep_dict[order.id]
        time_window_arr.append((order.ready_time, max_shop_dep))
    for order in all_orders:
        min_cust_arr = min_cust_arr_dict[order.id]
        time_window_arr.append((min_cust_arr, order.deadline))

    return time_window_arr


def print_solution_simple(data, manager, routing, solution, all_riders, K):
    for rider in all_riders:
        if rider.type == 'CAR':
            car_rider = rider
        elif rider.type == 'BIKE':
            bike_rider = rider
        else:
            walk_rider = rider

    """Prints solution on console."""
    # print(f"Objective: {solution.ObjectiveValue()}")
    total_cost = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_time = 0
        route_distance = 0
        prev_real_seq = manager.IndexToNode(index)

        while not routing.IsEnd(index):
            real_seq = manager.IndexToNode(index)

            plan_output += f" {real_seq} -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_time += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
            if prev_real_seq != real_seq:
                distance = data["distance_matrix"][prev_real_seq][real_seq]
                route_distance += distance
            prev_real_seq = real_seq

        if route_time > 0:
            plan_output += f"0\n"
            plan_output += f"Time of the route: {route_time}\n"
            plan_output += f"Distance of the route: {route_distance}\n"

            vehicle_type = data['vehicle_type_by_index'][vehicle_id]
            if vehicle_type == 'CAR':
                route_cost = car_rider.calculate_cost(route_distance)
            elif vehicle_type == 'BIKE':
                route_cost = bike_rider.calculate_cost(route_distance)
            else:
                route_cost = walk_rider.calculate_cost(route_distance)
            plan_output += f"Cost of the route: {route_cost}\n"

            # print(plan_output)

            total_cost += route_cost

    print(f"Total Cost of all routes: {total_cost}")
    print(f"Best Obj: {total_cost / K}")


def make_solution_bundle(data, manager, routing, solution):
    """Prints solution on console."""
    # print(f"Objective: {solution.ObjectiveValue()}")
    total_time = 0

    solution_bundle_arr = []
    order_size = len(data["pickups_deliveries"])

    for vehicle_id in range(data["num_vehicles"]):

        index = routing.Start(vehicle_id)
        vehicle_type = data["vehicle_type_by_index"][vehicle_id]

        shop_seq_arr = []
        dlv_seq_arr = []
        while not routing.IsEnd(index):
            real_seq = manager.IndexToNode(index)
            if real_seq != 0:
                if index <= order_size:
                    shop_seq = real_seq - 1
                    shop_seq_arr.append(shop_seq)
                else:
                    dlv_seq = real_seq - order_size - 1
                    dlv_seq_arr.append(dlv_seq)

            index = solution.Value(routing.NextVar(index))

        if len(shop_seq_arr) > 0:
            solution_bundle = [vehicle_type, shop_seq_arr, dlv_seq_arr]
            solution_bundle_arr.append(solution_bundle)

    customer_count = 0
    for solution_bundle in solution_bundle_arr:
        customer_count += len(solution_bundle[1])

    if customer_count != order_size:
        raise Exception("Short")

    return solution_bundle_arr
