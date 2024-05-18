from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import argparse
import numpy as np

SCALE_FACTOR = 10  # theoritically, this could be any number as long as it's greater than 10, but for the sake of simplicity, we will use 10

EARLY_DELIVERY_COST = 0.2 * SCALE_FACTOR
LATE_DELIVERY_COST = 1 * SCALE_FACTOR


def parse_vrp_instance(file_path, num_vehicles=1):
    with open(file_path, "r") as file:
        lines = file.readlines()

    vehicle_capacity = int(lines[1])

    start_index = 0
    for i, line in enumerate(lines):
        if "CUST NO." in line:
            start_index = i + 1  # Data starts after headers
            break

    locations = []
    demands = []
    time_windows = []
    service_times = []

    for line in lines[start_index:]:
        if line.strip():
            parts = line.split()
            cust_no, x, y, demand, ready_time, due_date, service_time = map(int, parts)
            locations.append((x, y))
            demands.append(demand)
            time_windows.append((ready_time, due_date))
            service_times.append(service_time)

    return {
        "locations": locations,
        "demands": demands,
        "time_windows": time_windows,
        "service_times": service_times,
        "vehicle_capacities": [vehicle_capacity] * num_vehicles,
        "num_vehicles": num_vehicles,
        "depot": 0,  # depot is always at index 0, could change but who knows...
    }


def print_solution(data, manager, routing, solution, time_matrix, distance_matrix, fixed_cost):
    """
    Route #1: 5 3 7 8 10 11 9 6 4 2 1 75
    Route #2: 13 17 18 19 15 16 14 12
    Route #3: 20 24 25 27 29 30 28 26 23 22 21
    Route #4: 32 33 31 35 37 38 39 36 34
    Route #5: 43 42 41 40 44 46 45 48 51 50 52 49 47
    Route #6: 57 55 54 53 56 58 60 59
    Route #7: 67 65 63 62 74 72 61 64 68 66 69
    Route #8: 81 78 76 71 70 73 77 79 80
    Route #9: 90 87 86 83 82 84 85 88 89 91
    Route #10: 98 96 95 94 92 93 97 100 99
    Cost 827.3
    """

    current_vehicle_index = 1

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        index = solution.Value(routing.NextVar(index))

        plan_output = f"Route #{current_vehicle_index}: "
        vehicle_used = False

        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)

            plan_output += f"{node_index} "

            vehicle_used = True

            index = solution.Value(routing.NextVar(index))  # Move to the next index/node in the route

        if vehicle_used:
            print(plan_output)
            current_vehicle_index += 1

    print("Objective: {}".format(solution.ObjectiveValue() / SCALE_FACTOR))


# Not really important. Will have to change this anyways.
def print_solution_debug(data, manager, routing, solution, time_matrix, distance_matrix, fixed_cost):

    total_load = 0
    total_distance = 0
    total_time = 0
    total_num_vehicles = 0

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = "Route for vehicle {}:\n".format(vehicle_id)

        route_load = 0

        first_node = True  # Indicator for the first node in the route

        distance_dimension = routing.GetDimensionOrDie("Distance")
        time_dimension = routing.GetDimensionOrDie("Time")

        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)

            if first_node:
                plan_output += " {0} (Depot) -> ".format(node_index)
                first_node = False
            else:
                route_load += data["demands"][node_index]

                plan_output += " {0} Load({1}) Distance({2}) Time({3}) -> ".format(
                    node_index,
                    route_load,
                    solution.Value(distance_dimension.CumulVar(node_index)) / SCALE_FACTOR,
                    solution.Value(time_dimension.CumulVar(node_index)) / SCALE_FACTOR,
                )

            prev_node_index = node_index
            index = solution.Value(routing.NextVar(index))  # Move to the next index/node in the route

        node_index = manager.IndexToNode(index)

        plan_output += " {0}\n".format(
            node_index,
        )

        print(plan_output)

    print("Fixed Cost: {}".format(fixed_cost / SCALE_FACTOR))
    print("Objective: {}".format(solution.ObjectiveValue() / SCALE_FACTOR))
    print("Total Number of used vehicles: {}".format(total_num_vehicles))


def compute_euclidean_distance_matrix(locations):
    distances = np.zeros((len(locations), len(locations)))

    for from_index, from_xy in enumerate(locations):
        from_x, from_y = from_xy

        for to_index, to_xy in enumerate(locations):
            to_x, to_y = to_xy

            # Here we use *10 followed by /10 and int() to truncate to 1 decimal place
            distances[from_index][to_index] = int((((from_x - to_x) ** 2 + (from_y - to_y) ** 2) ** 0.5) * 10) / 10

    return distances


def main(file_path, num_vehicles, time_limit):
    data = parse_vrp_instance(file_path=file_path, num_vehicles=num_vehicles)

    # Usuall stuff
    manager = pywrapcp.RoutingIndexManager(len(data["locations"]), data["num_vehicles"], data["depot"])

    routing = pywrapcp.RoutingModel(manager)

    distance_matrix = compute_euclidean_distance_matrix(data["locations"])
    time_matrix = np.copy(distance_matrix)

    distance_matrix_scaled = np.copy(distance_matrix) * SCALE_FACTOR
    time_matrix_scaled = np.copy(time_matrix) * SCALE_FACTOR
    service_times_scaled = np.array(data["service_times"]) * SCALE_FACTOR

    distance_matrix_scaled_list = np.copy(distance_matrix_scaled).astype(int).tolist()
    time_matrix_scaled_list = time_matrix_scaled.astype(int).tolist()
    service_times_scaled_list = service_times_scaled.astype(int).tolist()

    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        travel_time = time_matrix_scaled_list[from_node][to_node] + service_times_scaled_list[from_node]
        return travel_time


    time_callback_index = routing.RegisterTransitCallback(time_callback)
    transit_callback_index = routing.RegisterTransitMatrix(distance_matrix_scaled_list)

    # Include the total distance travelled in the objective function
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity",
    )

    distance = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # allow waiting time
        99999,  # maximum time per vehicle # TODO adjust?
        True,  # Don't force start cumul to zero, allow wait time.
        distance,
    )

    distance_dimension = routing.GetDimensionOrDie(distance)

    time = "Time"
    horizon = 10000  # waiting time allowed to vehicles. We do not want to "hard" constraint waiting, so a big number
    routing.AddDimension(
        time_callback_index,
        horizon,  # allow waiting time
        99999,  # maximum time per vehicle # TODO adjust?
        False,  # Don't force start cumul to zero, allow wait time.
        time,
    )

    time_dimension = routing.GetDimensionOrDie(time)

    for vehicle in range(data["num_vehicles"]):
        routing.AddVariableMinimizedByFinalizer(distance_dimension.CumulVar(routing.Start(vehicle)))
        routing.AddVariableMinimizedByFinalizer(distance_dimension.CumulVar(routing.End(vehicle)))

        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(vehicle)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(vehicle)))

    # Setting time windows
    for location_idx, time_window in enumerate(data["time_windows"]):
        index = manager.NodeToIndex(location_idx)
        start = int(time_window[0] * SCALE_FACTOR)
        end = int(time_window[1] * SCALE_FACTOR)
        window_length = end - start
        slack = int(0.2 * window_length)
        time_dimension.CumulVar(index).SetRange(start - slack, end + slack)
        time_dimension.SetCumulVarSoftLowerBound(index, start, EARLY_DELIVERY_COST)
        time_dimension.SetCumulVarSoftUpperBound(index, end, LATE_DELIVERY_COST)


    max_distance = max([max(row) for row in distance_matrix_scaled_list])
    fixed_cost = int(max_distance * 2 * (len(data["locations"]) - 1))

    # Every vehicle has a fixed cost which is activates for vehicle_i if vehicle_i is used
    routing.SetFixedCostOfAllVehicles(fixed_cost)

    # These are the best, talking after amazing amount of trial and error
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = time_limit

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        print_solution(data, manager, routing, solution, time_matrix, distance_matrix, fixed_cost)
    else:
        print("no solution found :(", routing.status())


if __name__ == "__main__":
    # Cool cs stuff
    parser = argparse.ArgumentParser()
    parser.add_argument("--file-path", type=str, required=True)
    parser.add_argument("--num-vehicles", type=int, required=True)
    parser.add_argument("--time-limit", type=int, required=True)
    args = parser.parse_args()

    main(file_path=args.file_path, num_vehicles=args.num_vehicles, time_limit=args.time_limit)
