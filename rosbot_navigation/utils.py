#!/usr/bin/env python3

import math
import numpy as np

def euclidean_distance(x1, y1, x2, y2):
    """Calculate Euclidean distance between two points."""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def is_close_to_point(current_x, current_y, target_x, target_y, threshold=0.3):
    """Check if current position is close to target position."""
    return euclidean_distance(current_x, current_y, target_x, target_y) < threshold

def get_closest_unknown_cell(occupancy_grid, robot_position, map_info):
    """Find the closest unknown cell in the occupancy grid."""
    # Convert robot position to grid coordinates
    grid_x = int((robot_position[0] - map_info.origin.position.x) / map_info.resolution)
    grid_y = int((robot_position[1] - map_info.origin.position.y) / map_info.resolution)
    
    # Get dimensions of the grid
    width = map_info.width
    height = map_info.height
    
    # Create a set to store visited cells during the search
    visited = set()
    
    # Queue for breadth-first search (BFS)
    queue = [(grid_x, grid_y, 0)]  # (x, y, distance)
    visited.add((grid_x, grid_y))
    
    # Define directions for neighbors (4-connectivity)
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    
    # Find the closest unknown cell using BFS
    while queue:
        x, y, dist = queue.pop(0)
        
        # Check if this cell is unknown (-1 value in occupancy grid)
        current_cell = y * width + x
        if 0 <= current_cell < len(occupancy_grid) and 0 <= x < width and 0 <= y < height:
            if occupancy_grid[current_cell] == -1:
                # Convert back to world coordinates
                world_x = x * map_info.resolution + map_info.origin.position.x
                world_y = y * map_info.resolution + map_info.origin.position.y
                return (world_x, world_y)
        
        # Add neighbors to the queue
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if (0 <= nx < width and 0 <= ny < height and 
                (nx, ny) not in visited):
                queue.append((nx, ny, dist + 1))
                visited.add((nx, ny))
    
    # If no unknown cell is found, return None
    return None

def find_frontiers(occupancy_grid, width, height, resolution, origin):
    """
    Find frontier points (borders between free and unknown space)
    Returns a list of potential exploration targets as world coordinates
    """
    frontiers = []
    
    # Define what we consider free, occupied, and unknown
    FREE_THRESHOLD = 20      # Values below this are considered free space
    OCCUPIED_THRESHOLD = 80  # Values above this are considered occupied
    UNKNOWN = -1
    
    # Check each cell in the grid
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            # Get the index in the flattened occupancy grid
            index = y * width + x
            
            # Skip if this cell is not free
            if occupancy_grid[index] >= FREE_THRESHOLD or occupancy_grid[index] == UNKNOWN:
                continue
                
            # Check if this free cell is adjacent to unknown space
            is_frontier = False
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    
                    neighbor_index = (y + dy) * width + (x + dx)
                    if 0 <= neighbor_index < len(occupancy_grid) and occupancy_grid[neighbor_index] == UNKNOWN:
                        is_frontier = True
                        break
                
                if is_frontier:
                    break
            
            if is_frontier:
                # Convert to world coordinates
                world_x = x * resolution + origin.position.x
                world_y = y * resolution + origin.position.y
                frontiers.append((world_x, world_y))
    
    # Remove frontiers that are too close to each other
    filtered_frontiers = []
    min_distance = 0.5  # Minimum distance between frontiers in meters
    
    for frontier in frontiers:
        is_valid = True
        for existing in filtered_frontiers:
            if euclidean_distance(frontier[0], frontier[1], existing[0], existing[1]) < min_distance:
                is_valid = False
                break
        
        if is_valid:
            filtered_frontiers.append(frontier)
    
    return filtered_frontiers

def cluster_frontiers(frontiers, cluster_tolerance=1.0):
    """
    Group nearby frontier points into clusters and return the centroid of each cluster
    """
    if not frontiers:
        return []
    
    clusters = []
    processed = set()
    
    for i, frontier in enumerate(frontiers):
        if i in processed:
            continue
            
        # Start a new cluster
        current_cluster = [frontier]
        processed.add(i)
        
        # Find all points close to this frontier
        for j, other_frontier in enumerate(frontiers):
            if j in processed:
                continue
                
            if euclidean_distance(frontier[0], frontier[1], 
                                  other_frontier[0], other_frontier[1]) < cluster_tolerance:
                current_cluster.append(other_frontier)
                processed.add(j)
        
        # Calculate the centroid of this cluster
        x_sum = sum(point[0] for point in current_cluster)
        y_sum = sum(point[1] for point in current_cluster)
        
        centroid = (x_sum / len(current_cluster), y_sum / len(current_cluster))
        clusters.append(centroid)
    
    return clusters
