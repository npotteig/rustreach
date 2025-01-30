import numpy as np
import csv
from tqdm import tqdm

USE_BICYCLE = True
    
def generate_start_goal_pairs(n, 
                              obstacles, 
                              width, 
                              vehicle_width, 
                              vehicle_height,
                              start_bounds,
                              goal_bounds):
    start_goal_pairs = []
    for _ in tqdm(range(n)):
        start, goal = generate_feasible_start_goal(obstacles, width, vehicle_width, vehicle_height, start_bounds, goal_bounds)
        start_goal_pairs.append((start, goal))
    return start_goal_pairs

def generate_feasible_start_goal(obstacles, width, vehicle_width, vehicle_height, start_bounds, goal_bounds):
    start = np.array([np.random.uniform(start_bounds[0][0], start_bounds[0][1]), np.random.uniform(start_bounds[1][0], start_bounds[1][1])])
    goal = np.array([np.random.uniform(goal_bounds[0][0], goal_bounds[0][1]), np.random.uniform(goal_bounds[1][0], goal_bounds[1][1])])
    while not is_feasible(start, goal, obstacles, width, vehicle_width, vehicle_height):
        start = np.array([np.random.uniform(start_bounds[0][0], start_bounds[0][1]), np.random.uniform(start_bounds[1][0], start_bounds[1][1])])
        goal = np.array([np.random.uniform(goal_bounds[0][0], goal_bounds[0][1]), np.random.uniform(goal_bounds[1][0], goal_bounds[1][1])])
    return start, goal

def is_feasible(start, goal, obstacles, width, vehicle_width, vehicle_height):
    for obstacle in obstacles:
        if is_collision(start, goal, obstacle, width, vehicle_width, vehicle_height):
            return False
    return True

def is_collision(start, goal, obstacle, width, vehicle_width, vehicle_height):
    # Divide the line into 100 points
    points = np.linspace(start, goal, 100)
    
    for point in points:
        if is_collision_point(point, obstacle, width, vehicle_width, vehicle_height):
            return True
    return False

def is_collision_point(point, obstacle, width, vehicle_width, vehicle_height):
    x, y = point
    x_obstacle, y_obstacle = obstacle
    if x_obstacle - width/2 <= x <= x_obstacle + width/2 and y_obstacle - width/2 <= y <= y_obstacle + width/2:
        return True
    if x_obstacle - vehicle_width/2 <= x <= x_obstacle + vehicle_width/2 and y_obstacle - vehicle_height/2 <= y <= y_obstacle + vehicle_height/2:
        return True
    return False

if __name__ == '__main__':
    n = 1000
    
    if USE_BICYCLE:
        save_path = 'eval_input_data/bicycle/corr_dataset.csv'
    else: 
        save_path = 'eval_input_data/quadcopter/corr_dataset.csv'
    
    obstacles = [[2.,0.7], [2., 1.4], [2., 1.9], [2., 2.4], [2., -0.7], [2., -1.4], [2., -1.9], [2., -2.4]]
    width = 0.5
    
    start_bounds = [[-0.5, 0.5], [-0.5, 0.5]]
    goal_bounds = [[3.5, 4.5], [-0.5, 0.5]]
    
    vehicle_start_bounds = [-0.5, 0.5]
    if USE_BICYCLE:
        vehicle_width = 0.7 # buffer of 0.1 on each side
        vehicle_height = 0.5 # buffer of 0.1 on each side
    else:
        vehicle_width = 0.52 # buffer of 0.1 on each side
        vehicle_height = 0.52 # buffer of 0.1 on each side
    
    vehicle_start_locations = np.random.uniform(vehicle_start_bounds[0], vehicle_start_bounds[1], (n, 2))
    start_goal_pairs = generate_start_goal_pairs(n, obstacles, width, vehicle_width, vehicle_height, start_bounds, goal_bounds)
    
    writer = csv.writer(open(save_path, 'w'))
    
    writer.writerow(['vehicle_x', 'vehicle_y', 'start_x', 'start_y', 'goal_x', 'goal_y'])
    
    for i in range(n):
        start, goal = start_goal_pairs[i]
        writer.writerow([*vehicle_start_locations[i], *start, *goal])