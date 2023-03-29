#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vrp_plot.py
Simple Vehicle Routing Problem with display using plotlib
In the *Vehicle Routing Problem (VRP)*, the goal is to find optimal routes for multiple vehicles visiting a set of locations. (When there's only one vehicle, it reduces to the Traveling Salesman Problem.)
This example of a VRP in which the goal is to minimize the longest single route.
Imagine a company that needs to visit its customers in a city made up of identical rectangular blocks.
Note that the location coordinates are not included in the problem data:
* all you need to solve the problem is the distance matrix, which we have pre-computed.
You only need the location data to identify the locations in the solution, which are denoted by their indices (0, 1, 2 ...)
in the above list.
The main purpose of showing the location coordinates and the city diagram is to provide a visual display of the problem and
its solution.
But this is not essential for solving a VRP.
For convenience in setting up the problem, the distances between locations are calculated using
[Manhattan distance](https://en.wikipedia.org/wiki/Taxicab_geometry), in which the distance between two points,
$(x_1, x_2)$ and $(y_1, y_2)$ is defined to be $|x_1 - x_2| + |y_1 - y_2|$.
However, there is no special reason to use this definition.
You can use whatever method is best suited to your problem to calculate distances.
"""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math
from matplotlib import pyplot as plt


# Add the solution printer
def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    max_route_distance = 0
    for vehicle_id in range(manager.GetNumberOfVehicles()):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print('Maximum of the route distances: {}m'.format(max_route_distance))


# Some tooling to ease drawing
height = 7

def plot_location(location, axes, color, location_number):
  axes.scatter(
      location[0],
      location[1],
      s=1000,
      facecolors='white',
      edgecolors=color,
      linewidths=2)
  axes.scatter(
      location[0],
      location[1],
      s=400,
      marker=f'${location_number}$',
      edgecolors=color,
      facecolors=color)

# A diagram of the city is shown below, with the company location marked in black and the locations to visit in blue.
def plot_locations(locations):
  fig, axes = plt.subplots(figsize=(1.7 * height, height))
  axes.grid(True)
  axes.set_xticks(list(set([x for (x, y) in locations])))
  axes.set_xticklabels([])
  axes.set_yticks(list(set([y for (x, y) in locations])))
  axes.set_yticklabels([])
  axes.set_axisbelow(True)
  for (i, location) in enumerate(locations):
    color = 'blue' if i else 'black'
    plot_location(location, axes, color, i)

# a diagram of the solution using a Google colorscheme
def plot_solution(locations, manager, routing, solution, loc, marker_size):
  height = 8
  fig, axes = plt.subplots(figsize=(1.7 * height, height))
  axes.grid(True)
  axes.set_xticks(list(set([x for (x, y) in locations])))
  axes.set_xticklabels([])
  axes.set_yticks(list(set([y for (x, y) in locations])))
  axes.set_yticklabels([])
  axes.set_axisbelow(True)
  max_route_distance = 0
  google_colors = [
      r'#4285F4', r'#EA4335', r'#FBBC05', r'#34A853', r'#101010', r'#FFFFFF'
  ]
  for vehicle_id in range(manager.GetNumberOfVehicles()):
    previous_index = routing.Start(vehicle_id)
    while not routing.IsEnd(previous_index):
      index = solution.Value(routing.NextVar(previous_index))
      start_node = manager.IndexToNode(previous_index)
      end_node = manager.IndexToNode(index)
      start = locations[start_node]
      end = locations[end_node]
      delta_x = end[0] - start[0]
      delta_y = end[1] - start[1]
      delta_length = math.sqrt(delta_x**2 + delta_y**2)
      unit_delta_x = delta_x / delta_length
      unit_delta_y = delta_y / delta_length
      axes.arrow(
          start[0] + (marker_size / 2) * unit_delta_x,
          start[1] + (marker_size / 2) * unit_delta_y,
          (delta_length - marker_size) * unit_delta_x,
          (delta_length - marker_size) * unit_delta_y,
          head_width=20,
          head_length=20,
          facecolor=google_colors[vehicle_id],
          edgecolor=google_colors[vehicle_id],
          length_includes_head=True,
          width=5)
      previous_index = index
      node_color = 'black' if routing.IsEnd(
          previous_index) else google_colors[vehicle_id]
      plot_location(end, axes, node_color, end_node)

# Create the 2D Locations
loc = [(456, 320), # location 0 - the depot
(228, 0),    # location 1
(912, 0),    # location 2
(0, 80),     # location 3
(114, 80),   # location 4
(570, 160),  # location 5
(798, 160),  # location 6
(342, 240),  # location 7
(684, 240),  # location 8
(570, 400),  # location 9
(912, 400),  # location 10
(114, 480),  # location 11
(228, 480),  # location 12
(342, 560),  # location 13
(684, 560),  # location 14
(0, 640),    # location 15
(798, 640)]  # location 16

#print(loc)
plot_locations(loc)
plt.show()

# Create the data
def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = [
        [
            0, 548, 776, 696, 582, 274, 502, 194, 308, 194, 536, 502, 388, 354,
            468, 776, 662
        ],
        [
            548, 0, 684, 308, 194, 502, 730, 354, 696, 742, 1084, 594, 480, 674,
            1016, 868, 1210
        ],
        [
            776, 684, 0, 992, 878, 502, 274, 810, 468, 742, 400, 1278, 1164,
            1130, 788, 1552, 754
        ],
        [
            696, 308, 992, 0, 114, 650, 878, 502, 844, 890, 1232, 514, 628, 822,
            1164, 560, 1358
        ],
        [
            582, 194, 878, 114, 0, 536, 764, 388, 730, 776, 1118, 400, 514, 708,
            1050, 674, 1244
        ],
        [
            274, 502, 502, 650, 536, 0, 228, 308, 194, 240, 582, 776, 662, 628,
            514, 1050, 708
        ],
        [
            502, 730, 274, 878, 764, 228, 0, 536, 194, 468, 354, 1004, 890, 856,
            514, 1278, 480
        ],
        [
            194, 354, 810, 502, 388, 308, 536, 0, 342, 388, 730, 468, 354, 320,
            662, 742, 856
        ],
        [
            308, 696, 468, 844, 730, 194, 194, 342, 0, 274, 388, 810, 696, 662,
            320, 1084, 514
        ],
        [
            194, 742, 742, 890, 776, 240, 468, 388, 274, 0, 342, 536, 422, 388,
            274, 810, 468
        ],
        [
            536, 1084, 400, 1232, 1118, 582, 354, 730, 388, 342, 0, 878, 764,
            730, 388, 1152, 354
        ],
        [
            502, 594, 1278, 514, 400, 776, 1004, 468, 810, 536, 878, 0, 114,
            308, 650, 274, 844
        ],
        [
            388, 480, 1164, 628, 514, 662, 890, 354, 696, 422, 764, 114, 0, 194,
            536, 388, 730
        ],
        [
            354, 674, 1130, 822, 708, 628, 856, 320, 662, 388, 730, 308, 194, 0,
            342, 422, 536
        ],
        [
            468, 1016, 788, 1164, 1050, 514, 514, 662, 320, 274, 388, 650, 536,
            342, 0, 764, 194
        ],
        [
            776, 868, 1552, 560, 674, 1050, 1278, 742, 1084, 810, 1152, 274,
            388, 422, 764, 0, 798
        ],
        [
            662, 1210, 754, 1358, 1244, 708, 480, 856, 514, 468, 354, 844, 730,
            536, 194, 798, 0
        ],
    ]
    data['num_vehicles'] = 4
    data['depot'] = 0
    return data

# Instantiate the data problem.
data = create_data_model()

# Create the routing index manager.
manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                        data['num_vehicles'], data['depot'])

# Create Routing Model.
routing = pywrapcp.RoutingModel(manager)

# Define the distance callback
def distance_callback(from_index, to_index):
    """Returns the distance between the two nodes."""
    # Convert from routing variable Index to distance matrix NodeIndex.
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data['distance_matrix'][from_node][to_node]

transit_callback_index = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Add a distance dimension
# To solve this VRP, you need to create a distance *dimension*, which computes the cumulative distance traveled by each vehicle along its route.
# You can then set a cost proportional to the maximum of the total distances along each route.
# Routing programs use dimensions to keep track of quantities that accumulate over a vehicle's route.
dimension_name = 'Distance'
routing.AddDimension(
    transit_callback_index,
    0,  # no slack
    3000,  # vehicle maximum travel distance
    True,  # start cumul to zero
    dimension_name)
distance_dimension = routing.GetDimensionOrDie(dimension_name)
distance_dimension.SetGlobalSpanCostCoefficient(100)

# Setting first solution heuristic.
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

# Solve the problem.
solution = routing.SolveWithParameters(search_parameters)

# Print solution on console.
if solution:
    print_solution(manager, routing, solution)
else:
    print('No solution found !')

plot_solution(loc, manager, routing, solution, loc,  50)
plt.show()