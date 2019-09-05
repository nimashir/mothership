import pandas as pd
import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import matplotlib.pyplot as plt
ROOTFOLDER = r'C:\Users\NS80482\Desktop\Personal\Mothership'

def compute_euclidean_distance_matrix(locations):
    """Creates callback to return distance between points
    borrowed from OR-tools
    """
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            elif to_counter ==0 :
                distances[from_counter][to_counter] = 0
            else:
                distances[from_counter][to_counter] = (int(
                    math.hypot((from_node[0] - to_node[0]),
                               (from_node[1] - to_node[1]))))
    return distances

def initialize_planet_asteroid_mapping(max_range, planet_id_list,asteroid_id_list,planet_locations_list,
                                       asteroid_locations_list):
    """This function initializes the proximity assignment of asteroids to planets. Each asteroids will
    be mapped to a planet if its distance to planet is less than a max_range."""
    planet_asteroid_mapping_dict = {}
    for i in range(len(planet_id_list)):
        planet_asteroid_mapping_dict[planet_id_list[i]] = []
        for j in range(len(asteroid_id_list)):
            dist = int(math.hypot((planet_locations_list[i][0] - asteroid_locations_list[j][0]),
                              (planet_locations_list[i][1] - asteroid_locations_list[j][1])))
            if dist<=max_range:
                planet_asteroid_mapping_dict[planet_id_list[i]].append(asteroid_id_list[j])
    return planet_asteroid_mapping_dict

def get_update_planet_asteroid_mapping(route_id_list, planet_id_list, planet_asteroid_mapping_dict):
    """This function updates the proximity assignment of asteroids to planets. At the end of each routing steps all the
    visited asteroids will be removed from the assignment list associated with each planet"""
    for id in route_id_list:
        for planet_id in planet_id_list:
            if id in planet_asteroid_mapping_dict[planet_id]:
                planet_asteroid_mapping_dict[planet_id].remove(id)
    return planet_asteroid_mapping_dict

def get_planet_proxy_astroid_count(planet_asteroid_mapping_dict):
    """This function return the number of asteroids in the proximity of each planet"""
    temp_dict = {}
    for planet_id in planet_asteroid_mapping_dict.keys():
        temp_dict[planet_id] = len(planet_asteroid_mapping_dict[planet_id])
    planet_proxy_cnt_df = pd.DataFrame({'id': list(temp_dict.keys()), 'count': list(temp_dict.values())})
    return planet_proxy_cnt_df

def filter_asteroids_around_current_planet(df, current_planet_coord, window_length,
                                           min_number_of_asteroids, magnifier):
    """TThis function filter and crop the search area. It gets the current planet coordination and window length and
    create  a square filter around the planet. If the number of asteroids within the filter boundary is less than
    min_number_of_asteroids, it increases the size of square edge using magnifier coefficient"""
    number_of_asteroids = 0
    current_depot_x = current_planet_coord[0]
    current_depot_y = current_planet_coord[1]
    while number_of_asteroids < min_number_of_asteroids:
        x_east = current_depot_x + window_length
        x_west = current_depot_x - window_length
        y_north = current_depot_y + window_length
        y_south = current_depot_y - window_length
        temp_df = df[(df.x <= x_east)&(df.x >= x_west)&
                     (df.y <= y_north)&(df.y >= y_south)]
        number_of_asteroids = len(temp_df)
        window_length = window_length * magnifier
    return temp_df

def get_locations_of_astroids(df, current_planet_coord, current_planet_id):
    """this function returns the coordination of all active asteroids within the filter along with
    the coordination of cuurent planet"""
    id_array = list(df['id'].values)
    x_array = df['x'].values
    y_array = df['y'].values
    locations = list(zip(x_array, y_array))
    locations.insert(0,current_planet_coord)
    id_array.insert(0,current_planet_id)
    return(locations, id_array)

def find_planet_with_closet_distance(sub_route, planet_df, asteroid_location_dict, planet_location_dict):
    """This function find the closet planet to the latest asteroid visited"""
    node_id = sub_route[-1]
    node_x , node_y = asteroid_location_dict[node_id]
    planet_df['distance'] = planet_df[['x','y']].apply(lambda row: (int(math.hypot((row['x'] - node_x),
                                                                                   (row['y'] - node_y)))), axis =1)
    min_distance_planet_id = planet_df['id'][planet_df['distance'].idxmin()]
    min_distance = planet_df['distance'].min()
    min_distance_planet_coord = planet_location_dict[min_distance_planet_id ]
    return (min_distance_planet_id, min_distance, min_distance_planet_coord)

def find_planet_with_maximum_utility(sub_route, planet_df, asteroid_location_dict, planet_location_dict,
                                     planet_proxy_count_df, distance_weight, count_weight):
    """This function find the planet with highest utility to go from the latest asteroid visited"""
    node_id = sub_route[-1]
    node_x , node_y = asteroid_location_dict[node_id]
    temp_df = planet_df.copy(deep = True)
    temp_df['distance'] = temp_df [['x','y']].apply(lambda row: (int(math.hypot((row['x'] - node_x),
                                                                                 (row['y'] - node_y)))), axis =1)
    temp_df = pd.merge(temp_df, planet_proxy_count_df, on='id')
    max_proxy_count = temp_df['count'].max()
    min_distance = temp_df['distance'].min()
    temp_df['utility'] = distance_weight * (min_distance/temp_df['distance']) +  \
                         count_weight * (temp_df['count']/max_proxy_count)
    max_utility_planet_id = temp_df['id'][temp_df['utility'].idxmax()]
    distance = temp_df['distance'][temp_df['id'] == max_utility_planet_id].values[0]
    max_utility_planet_coord = planet_location_dict[max_utility_planet_id]
    return (max_utility_planet_id, distance, max_utility_planet_coord)

def create_sub_route_from_current_planet(asteroid_subset_locations, vehicle_max_cap, id_array, penalty =10000):
    """This function calls or-tools and route the vehicle up to 20 asteroids"""
    demands = [1] * len(asteroid_subset_locations)
    demands[0] = 0
    data ={}
    data['locations'] = asteroid_subset_locations
    data['demands'] = demands
    data['vehicle_capacities'] = [vehicle_max_cap]
    data['num_vehicles'] = 1
    data['depot'] = 0
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['locations']), data['num_vehicles'], data['depot'])
    # Create distance matrix using asteriods coordination
    distance_matrix = compute_euclidean_distance_matrix(data['locations'])
    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)
    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    # Add Capacity constraint.
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
    # Allow to drop nodes.
    for node in range(1, len(distance_matrix)):
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)
    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    # Extract the solution
    return extract_CVRP_solution(manager, routing, assignment, id_array)

def extract_CVRP_solution(manager, routing, assignment, id_array):
    """This function returns solution of sub-routing"""
    node_list = []
    vehicle_id = 0
    index = routing.Start(vehicle_id)
    route_distance = 0
    while not routing.IsEnd(index):
        node_index = manager.IndexToNode(index)
        node_list.append(id_array[node_index])
        previous_index = index
        index = assignment.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
    node_index = manager.IndexToNode(index)
    node_list.append(id_array[node_index])
    node_list = node_list[:-1]
    return (node_list, route_distance)

def run_truck_routing_model(vehicle_max_cap, total_demand, window_length, min_number_of_asteroids, magnifier,
                            distance_weight, count_weight, max_range):
    """This is the Main module"""
    # read all data
    universe_df = pd.read_csv(ROOTFOLDER + '\\Data\\entities.csv')
    # filter planets' data
    planet_df = universe_df[universe_df['is_planet'] == True]
    # filter asteroids' data
    asteroid_df = universe_df[universe_df['is_planet'] == False]
    # create planet coordinate dictionary
    planet_locations_list = list(zip(planet_df['x'].values, planet_df['y'].values))
    planet_id_list = list(planet_df['id'].values)
    planet_location_dict = {key: value for key, value in zip(planet_id_list, planet_locations_list )}
    # create asteroid coordinate dictionary
    asteroid_locations_list = list(zip(asteroid_df['x'].values, asteroid_df['y'].values))
    asteroid_id_list = list(asteroid_df['id'].values)
    asteroid_location_dict = {key: value for key, value in zip(asteroid_id_list, asteroid_locations_list )}

    planet_asteroid_mapping_dict = initialize_planet_asteroid_mapping(max_range, planet_id_list,asteroid_id_list,
                                                                      planet_locations_list, asteroid_locations_list)
    master_route = []
    current_planet_coord = (0,0)
    current_planet_id = -1
    mined_asteroids = 0
    mining_round = 0
    traveled_distance = 0

    while mined_asteroids < total_demand:
        mining_round += 1
        filtered_asteroid_df = filter_asteroids_around_current_planet(asteroid_df, current_planet_coord,
                                                                      window_length, min_number_of_asteroids, magnifier)
        asteroid_subset_locations, id_array = get_locations_of_astroids(filtered_asteroid_df, current_planet_coord,
                                                                        current_planet_id)
        sub_route, sub_route_distance = create_sub_route_from_current_planet(asteroid_subset_locations, vehicle_max_cap,
                                                                             id_array, penalty=10000)
        mined_asteroids += (len(sub_route)-1)
        traveled_distance += sub_route_distance
        master_route.extend(sub_route)
        #update the current planet
        planet_asteroid_mapping_dict = get_update_planet_asteroid_mapping(sub_route, planet_id_list, planet_asteroid_mapping_dict)
        planet_proxy_count_df =  get_planet_proxy_astroid_count(planet_asteroid_mapping_dict)
        #min_distance_planet_id, min_distance, min_distance_planet_coord = find_closet_planet(sub_route, asteroid_df, planet_df)
        max_utility_planet_id, distance, max_utility_planet_coord = find_planet_with_maximum_utility(sub_route, planet_df, asteroid_location_dict, planet_location_dict,
                                         planet_proxy_count_df, distance_weight, count_weight)
        traveled_distance += distance
        current_planet_coord = max_utility_planet_coord
        current_planet_id = max_utility_planet_id
        # remove visited asteroids from the asteroid dataframe
        asteroid_df = asteroid_df[~ asteroid_df['id'].isin(sub_route)]
        print("Mining round: %s - Number of visited asteroids:%s - Distance traveled:%s"%
              (mining_round, len(master_route)- mining_round, traveled_distance ))

    return(master_route, traveled_distance)

def main():
    #Parameters
    vehicle_max_cap = 20
    total_demand = 2000
    min_number_of_asteroids = 30
    magnifier = 1.5
    window_length = 1500
    distance_weight = 0.5
    count_weight = 0.5
    max_range = 100
    # run the routing algorithm
    master_route, traveled_distance = run_truck_routing_model(vehicle_max_cap, total_demand, window_length, min_number_of_asteroids, magnifier,
                                distance_weight, count_weight, max_range)

    # plot the routing
    universe_df = pd.read_csv(ROOTFOLDER + '\\Data\\entities.csv')
    planet_df = universe_df[universe_df['is_planet'] == True]
    asteroid_df = universe_df[universe_df['is_planet'] == False]

    master_df = pd.DataFrame({'id': master_route})
    master_df = pd.merge(left=master_df, right=universe_df, on='id', how='left')
    master_df['x'][master_df['id'] == -1] = 0
    master_df['y'][master_df['id'] == -1] = 0
    master_df['is_planet'][master_df['id'] == -1] = False
    plt.figure(figsize=(10,10))
    plt.scatter(x = asteroid_df.x, y = asteroid_df.y, color='black', s=1)
    plt.scatter(x = planet_df.x, y = planet_df.y, color='red', s=4)
    plt.plot(list(master_df.x), list(master_df.y), linestyle='solid')
    print(traveled_distance)
    plt.show()

if __name__== "__main__":
  main()

