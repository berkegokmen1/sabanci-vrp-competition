from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

SCALE_FACTOR = 10


def parse_vrp_instance(file_path, num_vehicles=1):
    with open(file_path, "r") as file:
        lines = file.readlines()

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
        "vehicle_capacities": [200] * num_vehicles,
        "num_vehicles": num_vehicles,
        "depot": 0,
    }


def print_solution(data, manager, routing, solution):

    total_load = 0
    total_distance = 0
    total_time = 0
    total_num_vehicles = 0

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = "Route for vehicle {}:\n".format(vehicle_id)

        first_node = True  # Indicator for the first node in the route

        distance_dimension = routing.GetDimensionOrDie("Distance")
        time_dimension = routing.GetDimensionOrDie("Time")
        capacity_dimension = routing.GetDimensionOrDie("Capacity")

        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)

            if first_node:
                plan_output += " {0} (Depot) -> ".format(node_index)
                first_node = False
            else:
                plan_output += " {0} Load({1}) Distance({2}) Time({3}) -> ".format(
                    node_index,
                    solution.Value(capacity_dimension.CumulVar(index)),
                    solution.Value(distance_dimension.CumulVar(index)) / SCALE_FACTOR,
                    solution.Value(time_dimension.CumulVar(index)) / SCALE_FACTOR,
                )

            index = solution.Value(routing.NextVar(index))  # Move to the next index/node in the route

        plan_output += " {0} Load({1}) Distance({2}) Time({3})\n".format(
            node_index,
            solution.Value(capacity_dimension.CumulVar(index)),
            solution.Value(distance_dimension.CumulVar(index)) / SCALE_FACTOR,
            solution.Value(time_dimension.CumulVar(index)) / SCALE_FACTOR,
        )

        plan_output += "Load of the route: {}\n".format(solution.Value(capacity_dimension.CumulVar(index)))
        plan_output += "Distance of the route: {}\n".format(
            solution.Value(distance_dimension.CumulVar(node_index)) / SCALE_FACTOR
        )
        plan_output += "Time of the route: {}\n".format(
            solution.Value(time_dimension.CumulVar(node_index)) / SCALE_FACTOR
        )

        print(plan_output)

        total_load += solution.Value(capacity_dimension.CumulVar(index))
        total_distance += solution.Value(distance_dimension.CumulVar(index)) / SCALE_FACTOR
        total_time += solution.Value(time_dimension.CumulVar(index)) / SCALE_FACTOR
        total_num_vehicles += solution.Value(distance_dimension.CumulVar(index)) > 0

    print("Objective: {}".format(solution.ObjectiveValue() / SCALE_FACTOR))
    print("Total Load of all routes: {}".format(total_load))
    print("Total Distance of all routes: {}".format(total_distance))
    print("Total Time of all routes: {}".format(total_time))
    print("Total Number of used vehicles: {}".format(total_num_vehicles))


def compute_euclidean_distance_matrix(locations):
    distances = {}

    for from_index, from_xy in enumerate(locations):
        from_x, from_y = from_xy
        distances[from_index] = {}
        for to_index, to_xy in enumerate(locations):
            to_x, to_y = to_xy
            distances[from_index][to_index] = ((from_x - to_x) ** 2 + (from_y - to_y) ** 2) ** 0.5

    return distances


def compute_time_matrix(distance_matrix, service_times):
    time_matrix = distance_matrix.copy()

    for i in range(1, len(service_times)):
        for j in range(i + 1, len(service_times)):
            time_matrix[i][j] = time_matrix[j][i] = time_matrix[i][j] + service_times[j]

    return time_matrix


def calculate_fixed_cost(distance_matrix, num_customers):
    c_max = max(max(row.values()) for row in distance_matrix.values())
    return 2 * num_customers * c_max


def dict_to_list(d):
    # Parse to integer here, divide by 1 to truncate to integer
    # Later in the code when outputting something, divide by 10 to get the original value
    # Work with integers since that's the one supported here...
    return [[int(d[i][j] * SCALE_FACTOR) for j in d[i]] for i in d]


def main(file_path):
    data = parse_vrp_instance(file_path, num_vehicles=10)

    manager = pywrapcp.RoutingIndexManager(len(data["locations"]), data["num_vehicles"], data["depot"])

    routing = pywrapcp.RoutingModel(manager)

    distance_matrix = compute_euclidean_distance_matrix(data["locations"])
    time_matrix = compute_time_matrix(distance_matrix, data["service_times"])

    transit_callback_index = routing.RegisterTransitMatrix(dict_to_list(distance_matrix))
    time_callback_index = routing.RegisterTransitMatrix(dict_to_list(time_matrix))

    # routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    demand_callback_index = routing.RegisterUnaryTransitVector(data["demands"])
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

    for vehicle in range(data["num_vehicles"]):
        routing.AddVariableMinimizedByFinalizer(distance_dimension.CumulVar(routing.Start(vehicle)))
        routing.AddVariableMinimizedByFinalizer(distance_dimension.CumulVar(routing.End(vehicle)))

    time = "Time"
    horizon = 10000  # waiting time bruh
    routing.AddDimension(
        time_callback_index,
        horizon,  # allow waiting time
        99999,  # maximum time per vehicle # TODO adjust?
        False,  # Don't force start cumul to zero, allow wait time.
        time,
    )

    time_dimension = routing.GetDimensionOrDie(time)

    for vehicle in range(data["num_vehicles"]):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(vehicle)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(vehicle)))

    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx == data["depot"]:
            continue

        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0] * SCALE_FACTOR, time_window[1] * SCALE_FACTOR)

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            data["time_windows"][0][0] * SCALE_FACTOR, data["time_windows"][0][1] * SCALE_FACTOR
        )

    # Everything is multiplied by 10 to work with integers and be consistent
    routing.SetFixedCostOfAllVehicles(
        int(
            calculate_fixed_cost(distance_matrix=distance_matrix, num_customers=len(data["locations"]) - 1)
            * SCALE_FACTOR
        )
    )

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 10

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print("no solution found xd", routing.status())


if __name__ == "__main__":
    main(file_path="instance_1.txt")
