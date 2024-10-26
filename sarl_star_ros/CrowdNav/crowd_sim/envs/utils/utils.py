import numpy as np
from numpy.linalg import norm


def point_to_segment_dist(x1, y1, x2, y2, x3, y3):
    """
    Calculate the closest distance between point(x3, y3) and a line segment with two endpoints (x1, y1), (x2, y2)

    """
    px = x2 - x1
    py = y2 - y1

    if px == 0 and py == 0:
        return np.linalg.norm((x3-x1, y3-y1))

    u = ((x3 - x1) * px + (y3 - y1) * py) / (px * px + py * py)

    if u > 1:
        u = 1
    elif u < 0:
        u = 0

    # (x, y) is the closest point to (x3, y3) on the line segment
    x = x1 + u * px
    y = y1 + u * py

    return np.linalg.norm((x - x3, y-y3))

def point_to_segment_closest(x1, y1, x2, y2, x3, y3):
    """
    Calculate the closest distance between point(x3, y3) and a line segment with two endpoints (x1, y1), (x2, y2)
    """
    
    px = x2 - x1
    py = y2 - y1

    if px == 0 and py == 0:
        return np.linalg.norm((x3-x1, y3-y1))

    u = ((x3 - x1) * px + (y3 - y1) * py) / (px * px + py * py)

    if u > 1:
        u = 1
    elif u < 0:
        u = 0

    # (x, y) is the closest point to (x3, y3) on the line segment
    x = x1 + u * px
    y = y1 + u * py

    return x, y

def point_to_segment(x1, y1, x2, y2):
    """
    Return a random point on the line described by points (x1, y1) and (x2, y2).
    """
    # Generate a random value for parameter 'u'
    u = np.random.uniform(0, 1)

    # Calculate the coordinates of the random point on the line segment
    x = x1 + u * (x2 - x1)
    y = y1 + u * (y2 - y1)

    return x, y

def get_goals_in_line(p1, p2, n):
    """
    Generate n points evenly spaced along the line defined by p1 and p2.
    
    Args:
        p1 (tuple): Coordinates of the first point (x1, y1).
        p2 (tuple): Coordinates of the second point (x2, y2).
        n (int): Number of points to generate.
    
    Returns:
        list: A list containing tuples representing the generated points.
    """
    points = []
    for i in range(n):
        # Calculate the interpolation parameter
        t = float(i) / (n - 1)
        # Interpolate between the points
        x = p1[0] * (1 - t) + p2[0] * t
        y = p1[1] * (1 - t) + p2[1] * t
        points.append((x, y))
    return points

def rectangle_to_arc_dist(center_rect, length, width, center_cir, radius, initial_a, final_a):
    center_rect_arc_dist = point_to_arc_dist(center_cir, radius, initial_a, final_a, center_rect)
    print(center_rect_arc_dist)

    if center_rect_arc_dist[1] > 0:

        if center_rect[1] > center_cir[1]:
            x = center_rect[0] + width/2
            y = center_rect[1] - length/2
            (point, dist) = point_to_arc_dist(center_cir, radius, initial_a, final_a, (x, y))

        elif center_rect[1] == center_cir[1]:
            x = center_rect[0] + width/2
            y = center_rect[1]
            (point, dist) = point_to_arc_dist(center_cir, radius, initial_a, final_a, (x,y))

        else:
            x = center_rect[0] + width/2
            y = center_rect[1] + length/2
            (point, dist) = point_to_arc_dist(center_cir, radius, initial_a, final_a, (x,y))

    else:

        if center_rect[1] > center_cir[1]:
            x = center_rect[0] - width/2
            y = center_rect[1] + length/2
            
            (point, dist) = point_to_arc_dist(center_cir, radius, initial_a, final_a, (x, y))

        elif center_rect[1] == center_cir[1]:
            x = center_rect[0] - width/2
            y = center_rect[1] - length/2
            (point, dist) = point_to_arc_dist(center_cir, radius, initial_a, final_a, (x,y))

        else:
            x = center_rect[0] - width/2
            y = center_rect[1] - length/2
            (point, dist) = point_to_arc_dist(center_cir, radius, initial_a, final_a, (x,y))




    return (point, dist, (x, y))

def point_to_arc_dist(center, radius, init_a, final_a, position):

    angle_point_x = (position[0] - center[0])
    #print(angle_point_x)
    angle_point_y = (position[1] - center[1])
    #print(angle_point_y)
    angle_point = np.arctan2(angle_point_y, angle_point_x)
    #print(angle_point)

    if angle_point < (-1.0/2)*np.pi:
        angle_point = 2*np.pi + angle_point

    if abs(angle_point) >= init_a and abs(angle_point) <= final_a:
        dist = norm((position[0] - center[0], position[1] - center[1])) - radius
        point = (radius*np.cos(angle_point) + center[0], radius*np.sin(angle_point) + center[1])

    else:
        dist_1 = norm((position[0] - radius*np.cos(init_a) - center[0], position[1] - radius*np.sin(init_a) - center[1]))
        dist_2 = norm((position[0] - radius*np.cos(final_a) - center[0], position[1] - radius*np.sin(final_a) - center[1]))

        if dist_1 < dist_2:
            angle = init_a
        else:
            angle = final_a

        dist = min(dist_1, dist_2)
        point = (radius*np.cos(angle) + center[0], radius*np.sin(angle) + center[1])

    return (point, dist)

def check_rectangle_collision(circle_center, circle_radius, rectangle_center, rectangle_width, rectangle_height):
    """
    This function checks if a rectangle defined by its center, height, and width collides with a circle.

    Args:
        circle_center: A tuple representing the center coordinates (x, y) of the circle.
        circle_radius: The radius of the circle.
        rectangle_center: A tuple representing the center coordinates (x, y) of the rectangle.
        rectangle_width: The width of the rectangle.
        rectangle_height: The height of the rectangle.

    Returns:
        True if the rectangle collides with the circle, False otherwise.
    """

    # Find the distance between the circle center and the rectangle center
    distance_x = abs(circle_center[0] - rectangle_center[0])
    distance_y = abs(circle_center[1] - rectangle_center[1])

    # Consider the rectangle's half dimensions (half width and half height)
    half_width = rectangle_width / 2
    half_height = rectangle_height / 2

    # Check if the circle center is within the rectangle's sides (considering the rectangle's half dimensions)
    collision_x = distance_x <= half_width
    collision_y = distance_y <= half_height

    # If the center is within the rectangle, there's a collision
    if collision_x and collision_y:
        return True

    # Check for collision by considering the circle's radius and the distance to the rectangle's edges
    closest_x = max(rectangle_center[0] - half_width, min(circle_center[0], rectangle_center[0] + half_width))
    closest_y = max(rectangle_center[1] - half_height, min(circle_center[1], rectangle_center[1] + half_height))

    # Calculate the distance between the circle center and the closest point on the rectangle's edge
    distance_squared = (circle_center[0] - closest_x) ** 2 + (circle_center[1] - closest_y) ** 2

    return distance_squared <= circle_radius**2

# Helper function to calculate squared distance
def distance_squared(point1, point2):
    return (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2

def rectangle_in_concentric_circles(rectangle_center, rectangle_width, rectangle_height, circle_center, radius1, radius2):
    """
    This function checks if a rectangle defined by its center, width, and height is entirely within a region formed by two concentric circles.

    Args:
        rectangle_center: A tuple representing the center coordinates (x, y) of the rectangle.
        rectangle_width: The width of the rectangle.
        rectangle_height: The height of the rectangle.
        circle_center: A tuple representing the center coordinates (x, y) of the concentric circles.
        radius1: The radius of the inner circle.
        radius2: The radius of the outer circle (must be bigger than radius1).

    Returns:
        True if the rectangle is entirely within the concentric circle region, False otherwise.
    """

    # Check if the rectangle's center is within the outer circle
    if not (abs(rectangle_center[0] - circle_center[0]) <= radius2 and abs(rectangle_center[1] - circle_center[1]) <= radius2):
        return False

    # Consider rectangle's half dimensions (half width and half height) for corner checks
    half_width = rectangle_width / 2
    half_height = rectangle_height / 2

    # Loop through the rectangle's four corners and check if each corner is within both circles
    for corner_x in [rectangle_center[0] - half_width, rectangle_center[0] + half_width]:
        for corner_y in [rectangle_center[1] - half_height, rectangle_center[1] + half_height]:
            if (distance_squared((corner_x, corner_y), circle_center) > radius2**2 or
                distance_squared((corner_x, corner_y), circle_center) < radius1**2):
                return False

    # If all corners are within both circles, the rectangle is entirely inside
    return True

class RectangleObstacle:
    def __init__(self, center, width, height):
        self.center = center
        self.width = width
        self.height = height

    def contains(self, point):
        return (abs(point[0] - self.center[0]) < self.width / 2 and
                abs(point[1] - self.center[1]) < self.height / 2)

    def collides(self, other):
        return (abs(self.center[0] - other.center[0]) * 2 < (self.width + other.width) and
                abs(self.center[1] - other.center[1]) * 2 < (self.height + other.height))
    
    def get_vertices(self):
        x, y = self.center
        w, h = self.width / 2, self.height / 2
        return [(x - w, y - h), (x + w, y - h), (x + w, y + h), (x - w, y + h)]