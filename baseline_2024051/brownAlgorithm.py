import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


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

    # Create Routing Model.
    # [START routing_model]
    routing = pywrapcp.RoutingModel(manager)

    # [END routing_model]

    # Define cost of each arc.
    # [START arc_cost]
    def distance_callback(from_index, to_index):
        """Returns the manhattan distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
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

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        9999,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Define Transportation Requests.
    for request in data['pickups_deliveries']:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        routing.solver().Add(distance_dimension.CumulVar(pickup_index) <= distance_dimension.CumulVar(delivery_index))

    # Create and register a transit callback.
    def time_callback_car(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["time_matrix_car"][from_node][to_node]

    def time_callback_bike(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["time_matrix_bike"][from_node][to_node]

    def time_callback_walk(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["time_matrix_walk"][from_node][to_node]

    transit_callback_index_car = routing.RegisterTransitCallback(time_callback_car)
    transit_callback_index_bike = routing.RegisterTransitCallback(time_callback_bike)
    transit_callback_index_walk = routing.RegisterTransitCallback(time_callback_walk)

    for vehicle_index in range(len(data["vehicle_type_by_index"])):
        vehicle_type = data["vehicle_type_by_index"][vehicle_index]
        if vehicle_type == 'CAR':
            routing.SetArcCostEvaluatorOfVehicle(transit_callback_index_car, vehicle_index)
        elif vehicle_type == 'BIKE':
            routing.SetArcCostEvaluatorOfVehicle(transit_callback_index_bike, vehicle_index)
        else:
            routing.SetArcCostEvaluatorOfVehicle(transit_callback_index_walk, vehicle_index)

        # Add Time Windows constraint.
    time_window_car = "TimeCar"
    routing.AddDimension(
        transit_callback_index_car,
        0,  # allow waiting time
        9999,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time_window_car,
    )

    time_window_bike = "TimeBike"
    routing.AddDimension(
        transit_callback_index_bike,
        9999,  # allow waiting time
        30,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time_window_bike,
    )
    time_window_walk = "TimeWalk"
    routing.AddDimension(
        transit_callback_index_walk,
        9999,  # allow waiting time
        30,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time_window_walk,
    )
    time_dimension_car = routing.GetDimensionOrDie(time_window_car)
    time_dimension_bike = routing.GetDimensionOrDie(time_window_bike)
    time_dimension_walk = routing.GetDimensionOrDie(time_window_walk)

    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data["time_window"]):
        if location_idx == data["depot"]:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension_car.CumulVar(index).SetRange(0, time_window[1])


    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)
    # search_parameters.time_limit.seconds = 10
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    print(assignment)

    # Print solution on console.
    if assignment:
        print("1")
        print_solution(data, manager, routing, assignment)


def make_input_data(K, dist_mat, all_orders, all_riders):
    data = {}

    data["depot"] = 0  # dummy depot

    data["distance_matrix"] = make_distance_matrix(K, dist_mat)

    data["time_window"] = make_time_window(all_orders)

    data["pickups_deliveries"] = make_pickup_delivery(K)

    data["demands"] = make_demand(all_orders)

    data["vehicle_type_by_index"] = {}
    vehicle_capacity_arr = []
    num_vehicles = 0

    vehicle_index = 0
    for rider in all_riders:
        num_vehicles += rider.available_number
        for _ in range(rider.available_number):
            vehicle_capacity_arr.append(rider.capa)
            data["vehicle_type_by_index"][vehicle_index] = rider.type
            vehicle_index += 1

        time_matrix = [
            [int(x / rider.speed + rider.service_time) if x > 0 else 0 for x in row]
            for row in data["distance_matrix"]]

        if rider.type == 'CAR':
            data["time_matrix_car"] = time_matrix
        elif rider.type == 'BIKE':
            data["time_matrix_bike"] = time_matrix
        else:
            data["time_matrix_walk"] = time_matrix

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
    for row_index in range(K):
        for column_index in range(K):
            new_dist_matrix[row_index + 1][column_index + 1] = dist_mat[row_index][column_index]

    # set long distance from depot to customer
    for customer_index in range(K + 1, 2 * K + 1):
        new_dist_matrix[0][customer_index] = 999999

    # set long distance from customer to shop
    for customer_index in range(K + 1, 2 * K + 1):
        for shop_index in range(1, K + 1):
            if customer_index == 101 and shop_index == 100:
                t = 1
            new_dist_matrix[customer_index][shop_index] = 999999

    tolist = new_dist_matrix.astype(int).tolist()
    return tolist


def make_pickup_delivery(K):
    np_array = np.zeros((K, 2))
    for order_index in range(K):
        np_array[order_index][0] = int(order_index + 1)
        np_array[order_index][1] = int(order_index + 1 + K)
    return np_array.astype(int).tolist()


def make_time_window(all_orders):
    time_window_arr = []
    time_window_arr.append((0, 99999))

    for order in all_orders:
        time_window_arr.append((order.ready_time, order.deadline))
    for order in all_orders:
        time_window_arr.append((order.ready_time, order.deadline))

    return time_window_arr


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    max_route_distance = 0
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
        max_route_distance = max(route_distance, max_route_distance)
    print(f"Maximum of the route distances: {max_route_distance}m")
