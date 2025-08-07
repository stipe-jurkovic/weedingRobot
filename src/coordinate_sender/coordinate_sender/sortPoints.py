import math
        
def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def sort_points_greedy(points, start=(0,0)):
    points = points.copy()
    
    if len(points) < 2:
        return points
    
    # Find the nearest point to start, that becomes the initial current point
    current_point = min(points, key=lambda p: distance(p, start))
    sorted_path = [current_point]
    points.remove(current_point)
    
    while points:
        nearest = min(points, key=lambda p: distance(p, current_point))
        sorted_path.append(nearest)
        points.remove(nearest)
        current_point = nearest
    
    return sorted_path