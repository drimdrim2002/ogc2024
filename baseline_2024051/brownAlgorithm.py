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

    print(data)


def make_input_data(K, dist_mat, all_orders, all_riders):
    data = {}
    data["depot"] = 0

    data["distance_matrix"] = np.zeros((2 * K + 1, 2 * K + 1))
    for row_index in range(K):
        for column_index in range(K):
            data["distance_matrix"][row_index + 1][column_index + 1] = dist_mat[row_index][column_index]
    data["distance_matrix"] = data["distance_matrix"].tolist()


    data["pickups_deliveries"] = np.zeros((K, 2))
    for order_index in range(K):
        data["pickups_deliveries"][order_index][0] = int(order_index + 1)
        data["pickups_deliveries"][order_index][1] = int(order_index + 1 + K)
    data["pickups_deliveries"] = data["pickups_deliveries"].tolist()

    data["demands"] = []
    data["demands"].append(0)
    for order in all_orders:
        data["demands"].append(int(order.volume))

    data["vehicle_capacities"] = []
    for rider in all_riders:
        for _ in len(rider.available_number):
            data["vehicle_capacities"].append(rider.capa)


    return data


def make_distance_matrix(K, dist_mat):
    new_dist_matrix = np.zeros((2 * K + 1, 2 * K + 1))
    for row_index in range(K):
        for column_index in range(K):
            new_dist_matrix[row_index + 1][column_index + 1] = dist_mat[row_index][column_index]
    new_dist_matrix = new_dist_matrix.tolist()
    return new_dist_matrix