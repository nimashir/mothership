# Data Science Project Challenge

## Objective

To develop a solution algorithm to mine and sell 2000 asteroids with minimum traveled distance.
Overview of problem landscape
The dataset under study is comprised of 100 planets and 4673 asteroids. The following graph shows their spatial distribution of planets (red dot) and asteroids (black dot):
 
 ![Alt text](https://github.com/nimashir/mothership/blob/master/spatial_distribution.PNG)
 
 
 
 Clearly, our universe is not evenly distributed; there is a higher density of asteroids in upper right, compared to more dispersed density in lower left. In addition, density of asteroids is much higher in close proximity of planets and as you get farther from the planet there is lower density of asteroids.
 
## methodology

This problem is a variant of capacitated vehicle routing problem. The capacity of vehicle (20) is relatively small compared to the demand (2000). Therefore, at least 100 rounds of loading and unloading is required. For the purpose of vehicle routing within each loading, I have used OR-tools library. More specifically, I’ve designed my solution to use capacitated vehicle routing with penalty for unvisited demand. In designing the solution algorithm, the following considerations have been made:

*	The solution utilizes only one vehicle. 
*	The overall routing is constructed sequentially. In every step, the vehicle will be routed to fill its capacity(20 asteroid visits), then it will be directed to the planet with highest utility(will be discussed later)
*	Since the algorithm works sequentially, the solution space need to be decomposed to speed up the running time. At each step (except the first step), the vehicle starts from a planet. We limit the solution space for routing by applying a spatial filter and crop around the current planet. The idea is that when you are around (0,0), there is no point of having option to travel directly to (10,000,10,000)
*	At each steps and after filtering the asteroid candidates, all the asteroid would have a demand equal 1. Now if you have 50 asteroids, the vehicle would be able to visit only 20 asteroids, and the remaining 30 asteroids remain unvisited through adding penalty to the objective function.
*	At the end of each step, all visited asteroids will be dropped permanently from potential asteroids
*	At the end of each step, the vehicle returns to the planet with highest utility. The utility of planet is defined based on its distance to the location of vehicle at the last stop, and the density of asteroids around the planet.
*	In order to calculate the utility, the number of asteroids within some range of distance will be updated for each planet.  As a result, at the end of each iteration we have asteroid_count and distance for each planet; min distance and max_asteroid_count across the universe. The utility of each planet will be updated as 
Utility = distance_weight * (min_distance/distance) + count_weight * (asteroid_count/max_asteroid_count).

*	Note that at each step the visited asteroids will be deleted permanently, hence, the asteroid proximity will be constantly changing.
*	At the end of each step, OR-tools is set to return the vehicle to the planet from which the routing has started. We would ignore this return and the corresponding cost in the solution

## Solution

The algorithm has 8 parameters: 
*	vehicle_max_cap: given as 20
*	total_demand: given as 2000
*	min_number_of_asteroids: Given the max capacity of 20, having at least 30 candidate at each routing steps is reasonable
*	magnifier: is fixed as 1.5. With window length greater than 1000, we would have 30 candidates anyway
*	window_length: a range of [1500-2000] is suggested. With greater window length, the feasible space will be much larger and as a result the running time will be higher
*	distance_weight, count_weight: While not required but the sum of weights are suggested to be 1. As the count_weight increases(distance weight decreases), the vehicle would consider even farther planets for unloading
*	max_range: a range of (50,200) is suggested. With higher value, the impact of asteroid proximity on planet utility will be lower.

By conducting some sensitivity analysis, the best routing cost is found with distance weight of 0.5, count weight of 0.5, max range of 100, and window length of 1500. The corresponding cost is 113,596 and the running time is 65 seconds

 ![Alt text](https://github.com/nimashir/mothership/blob/master/routing_solution.PNG)
 
## Potential Extension

*	A backward approach: Starting from high density areas and moving toward (0,0) 
*	Multi-vehicle approach: Running multiple vehicle into the space and matching their solution subsequently.
*	Applying Mehta-heuristic methods such as genetic algorithm is another alternative. Especially, if coupled with multi-vehicle approach.
*	Looking for approaches to allow unloading at any stage rather than limiting the unloading to a full capacity vehicle








