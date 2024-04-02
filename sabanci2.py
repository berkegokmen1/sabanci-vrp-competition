#!/usr/bin/env python3
"""Small Vehicle routing problem with time, distance and capacity constraints."""
from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

distance_matrix = [
    [0, 0, 0, 0, 121, 47, 72, 25, 4],
    [0, 0, 0, 0, 121, 47, 72, 25, 4],
    [0, 0, 0, 0, 121, 47, 72, 25, 4],
    [0, 0, 0, 0, 121, 47, 72, 25, 4],
    [124, 124, 124, 124, 0, 89, 177, 137, 131],
    [50, 50, 50, 50, 92, 0, 102, 63, 56],
    [81, 81, 81, 81, 188, 114, 0, 90, 75],
    [40, 40, 40, 40, 137, 63, 92, 0, 37],
    [2, 2, 2, 2, 113, 38, 75, 28, 0],
]

time_matrix = [
    [0, 0, 0, 0, 14, 7, 11, 8, 2],
    [0, 0, 0, 0, 14, 7, 11, 8, 2],
    [0, 0, 0, 0, 14, 7, 11, 8, 2],
    [0, 0, 0, 0, 14, 7, 11, 8, 2],
    [14, 14, 14, 14, 0, 12, 20, 19, 16],
    [7, 7, 7, 7, 13, 0, 13, 11, 9],
    [11, 11, 11, 11, 22, 15, 0, 17, 13],
    [12, 12, 12, 12, 21, 13, 17, 0, 11],
    [1, 1, 1, 1, 13, 5, 12, 9, 0],
]


def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data["time_matrix"] = time_matrix
    data["time_windows"] = [
        (0, 1000),  # depot
        (0, 1000),  # unload depot 1
        (0, 1000),  # unload depot 2
        (0, 1000),  # unload depot 3
        (0, 1000),  # 1
        (0, 1000),  # 2
        (0, 1000),  # 3
        (0, 1000),  # 4
        (0, 1000),  # 5
    ]
    data["distance_matrix"] = distance_matrix
    data["demands"] = [0, -15, -15, -15, 10, 10, 10, 10, 10]  # 1  # 2  # 3  # 4  # 5
    data["vehicle_capacities"] = [15, 15, 15, 15]
    data["num_vehicles"] = 4
    data["depot"] = 0
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    total_distance = 0
    time_dimension = routing.GetDimensionOrDie("Time")
    capacity_dimension = routing.GetDimensionOrDie("Capacity")
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            load_var = capacity_dimension.CumulVar(index)
            route_load = solution.Value(load_var)
            plan_output += f" {node_index} Load({route_load}) -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        node_index = manager.IndexToNode(index)
        load_var = capacity_dimension.CumulVar(index)
        route_load = solution.Value(load_var)
        plan_output += f" {node_index} Load({route_load})\n"
        plan_output += f"Distance of the route: {route_distance}m\n"
        print(plan_output)
        total_distance += route_distance
    print(f"Total distance of all routes: {total_distance}m")


def main():
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data["distance_matrix"]), data["num_vehicles"], data["depot"])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        3000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name,
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    capacity = "Capacity"
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        15,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities
        True,  # start cumul to zero
        capacity,
    )
    capacity_dimension = routing.GetDimensionOrDie(capacity)
    # Set slack to zero for each location except depot.
    for location_idx in range(len(data["demands"])):
        index = manager.NodeToIndex(location_idx)
        if location_idx < 4:
            capacity_dimension.SlackVar(index).SetRange(0, 15)
        else:
            capacity_dimension.SlackVar(index).SetRange(0, 0)

    ############# Time ##############

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["time_matrix"][from_node][to_node]

    time_callback_index = routing.RegisterTransitCallback(time_callback)

    # Add Time Windows constraint.
    time = "Time"
    routing.AddDimension(
        time_callback_index,
        30,  # allow waiting time
        10000,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx == 0:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node.
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data["time_windows"][0][0], data["time_windows"][0][1])

    # Instantiate route start and end times to produce feasible times.
    for i in range(data["num_vehicles"]):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    ############# Time ##############

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.FromSeconds(1)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print(solution)


if __name__ == "__main__":
    main()
