import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math

BIG_PENALTY_VALUE = 999999


def algorithm(K, all_orders, all_riders, dist_mat, timelimit=60):
    print(f'K : {K}')

    for order in all_orders:
        print(f'Order : {order}')

    for rider in all_riders:
        print(f'Rider: {rider}')

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

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Create and register a transit callback.
    def time_callback_car(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["time_matrix_car"][from_node][to_node]

    transit_callback_index_car = routing.RegisterTransitCallback(time_callback_car)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index_car)

        # Add Time Windows constraint.
    time_window_car = "TimeCar"
    routing.AddDimension(
        transit_callback_index_car,
        0,  # allow waiting time
        BIG_PENALTY_VALUE - 1000,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time_window_car,
    )

    time_dimension_car = routing.GetDimensionOrDie(time_window_car)

    # Define Transportation Requests.
    for request in data['pickups_deliveries']:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        routing.solver().Add(time_dimension_car.CumulVar(pickup_index) <= time_dimension_car.CumulVar(delivery_index))

    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx == data["depot"]:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension_car.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)
    # search_parameters.time_limit.seconds = 60
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    # Print solution on console.
    if assignment:
        print_solution_simple(data, manager, routing, assignment)
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

            if vehicle_type == 'WALK':
                for dlvy_seq in dlvry_seq_by_type:
                    for seq in dlvy_seq:
                        print(f'Distance: {data["distance_matrix"][seq + 1][seq + K + 1]}')
                        print(f'Time: {data["time_matrix_walk"][seq + 1][seq + K + 1]}')

        return solution_bundle_arr
    else:
        print("No assignment")


def make_input_data(K, dist_mat, all_orders, all_riders):
    data = {}

    data["depot"] = 0  # dummy depot

    data["distance_matrix"] = make_distance_matrix(K, dist_mat)

    data["time_windows"] = make_time_window(all_orders)

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
                    time_matrix[row_index][column_index] = int(0)
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

    # print("BIKE")
    # for i in range(K + 1):
    #     print(data["time_matrix_bike"][i][i + 100])
    #
    # print("WALK")
    # for i in range(K + 1):
    #     print(data["time_matrix_walk"][i][i + 100])
    #
    # print("CAR")
    # for i in range(K + 1):
    #     print(data["time_matrix_car"][i][i + 100])

    for rider in all_riders:
        num_vehicles += rider.available_number
        for _ in range(rider.available_number):
            vehicle_capacity_arr.append(rider.capa)
            data["vehicle_type_by_index"][vehicle_index] = rider.type
            vehicle_index += 1

    data["num_vehicles"] = num_vehicles
    data["vehicle_capacities"] = vehicle_capacity_arr

    return data


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


def make_time_window(all_orders):
    time_window_arr = [(0, BIG_PENALTY_VALUE)]

    for order in all_orders:
        time_window_arr.append((order.ready_time, order.deadline - 1))
    for order in all_orders:
        time_window_arr.append((order.ready_time, order.deadline - 1))

    return time_window_arr


def print_solution_simple(data, manager, routing, solution):
    """Prints solution on console."""
    # print(f"Objective: {solution.ObjectiveValue()}")
    total_distance = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += f" {manager.IndexToNode(index)} -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        plan_output += f"{manager.IndexToNode(index)}\n"
        plan_output += f"Distance of the route: {route_distance}m\n"
        print(plan_output)
        total_distance += route_distance
    print(f"Total Distance of all routes: {total_distance}m")


def make_solution_bundle(data, manager, routing, solution):
    """Prints solution on console."""
    # print(f"Objective: {solution.ObjectiveValue()}")
    total_time = 0

    solution_bundle_arr = []
    order_size = len(data["pickups_deliveries"])

    for vehicle_id in range(data["num_vehicles"]):

        if vehicle_id == 1:
            t = 1
        index = routing.Start(vehicle_id)
        vehicle_type = data["vehicle_type_by_index"][vehicle_id]

        shop_seq_arr = []
        dlv_seq_arr = []
        while not routing.IsEnd(index):
            real_seq = manager.IndexToNode(index)
            if real_seq != 0:
                if index <= order_size:
                    if index == 101:
                        t = 1
                    shop_seq = real_seq - 1
                    shop_seq_arr.append(shop_seq)
                else:
                    dlv_seq = real_seq - order_size - 1
                    dlv_seq_arr.append(dlv_seq)

            index = solution.Value(routing.NextVar(index))

        if len(shop_seq_arr) != len(dlv_seq_arr):
            t = 1

        if len(shop_seq_arr) > 0:
            solution_bundle = []
            solution_bundle.append(vehicle_type)
            solution_bundle.append(shop_seq_arr)
            solution_bundle.append(dlv_seq_arr)
            solution_bundle_arr.append(solution_bundle)

    customer_count = 0
    for solution_bundle in solution_bundle_arr:
        customer_count += len(solution_bundle[1])

    if customer_count != order_size:
        raise Exception("Short")

    print(f"Total time of all routes: {total_time}min")
    return solution_bundle_arr
