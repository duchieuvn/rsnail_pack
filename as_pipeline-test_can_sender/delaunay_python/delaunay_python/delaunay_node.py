import datetime

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from rclpy.time import Time
from scipy.spatial import Delaunay
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import PointStamped, Vector3
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
# Constants
FACTOR_ANGLE = 1
MAX_ANGLE_CHANGE = 0.75
FACTOR_DISTANCE = 1
MAX_DISTANCE = 7
MID_DISTANCE = 5
MAX_LEVEL = 2
MAX_VIEWING_DISTANCE = 10
STEP_WIDTH = 2
POSITION = np.array([0, 0])
SELECTION = {
    "Blue Cones": 'blue',
    "Yellow Cones": 'yellow',
    "Big Orange Cones": 'orange',
    "Unknown Cones": 'black'
}

# Global midpoints (will be computed in the planner)
midpoints = {}

class PathNode:
    def __init__(self, point: np.ndarray, previous=None):
        self.point: np.ndarray = point
        self.previous: PathNode = previous
        self.cost: float = 0
        self.options: list[PathNode] = []

    def sort_paths(self):
        self.options.sort(key=lambda n: n.cost)

    def expand(self, depth: int):
        print('Expand called')
        if depth <= 0 :#or len(self.options):
            return

        self.cost = float("inf")
        neighbors = get_neighbors(np.array(self.point), np.array(self.previous.point) if self.previous else None)
        for point in neighbors:
            cost = apply_cost_function(point, self.point, self.previous.point if self.previous else self.point)
            if cost['normalized_angle'] > MAX_ANGLE_CHANGE or cost['normalized_distance'] > MAX_DISTANCE:
                continue

            new_path_node = PathNode(point, self)
            self.options.append(new_path_node)
            new_path_node.expand(depth - 1)
            self.cost = min(self.cost, new_path_node.cost + cost['total'])

        self.sort_paths()
        print(self.options)

    def extract_points(self, arr):
        if len(self.options) > 0:
            arr.append(self.point)
            arr = self.options[0].extract_points(arr)
        return arr

    def get_node(self, depth: int):
        if depth <= 0 or len(self.options) <= 0:
            return self
        else:
            next_node: PathNode = self.options[0]
            return next_node.get_node(depth - 1)


def triple_midpoint(point1, point2, point3):
    points = [[point1, point2], [point1, point3], [point2, point3]]
    res = []
    for p in points:
        diff_vector = p[0] - p[1]
        if np.hypot(diff_vector[0], diff_vector[1]) < 5:
            res.append(midpoint(p[0], p[1]))
    return res

def midpoint(p1, p2):
    return np.array([(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2])

def transform_structure(triangles):
    result = {}
    for t in triangles:
        for i, point in enumerate(t):
            other_points = list(t)
            point = tuple(point)
            if point in result.keys():
                # Concatenate tuple(s) of neighbor points
                result[point] = result[point] + (other_points[:i] + other_points[i + 1:],)
            else:
                result[point] = (other_points[:i] + other_points[i + 1:],)
    return result

def apply_cost_function(point, position, last_position):
    costs = {}
    costs['normalized_angle'] = FACTOR_ANGLE * ((get_angle(point, position, last_position)) / 180)
    deviation_distance = np.abs(get_distance(point, position) - MID_DISTANCE)
    costs['normalized_distance'] = FACTOR_DISTANCE * (1 if deviation_distance > MAX_DISTANCE else deviation_distance / MAX_DISTANCE)
    costs['total'] = costs['normalized_angle'] + costs['normalized_distance']
    return costs

def get_distance(point, position):
    return np.linalg.norm(point - position)

def get_angle(point, position, last_position):
    v1 = position - last_position
    v2 = point - position
    dot_product = np.dot(v1, v2)
    mag1 = np.linalg.norm(v1)
    mag2 = np.linalg.norm(v2)
    if mag1 == 0 or mag2 == 0:
        return 0
    angle = dot_product / (mag1 * mag2)
    angle = np.clip(angle, -1, 1)
    angle = np.arccos(angle)
    return np.rad2deg(angle)

def get_nearest_point(position):
    minimum_distance = float('inf')
    ret = None
    for point in midpoints.keys():
        val = get_distance(np.array(point), position)
        if val < minimum_distance:
            minimum_distance = val
            ret = point
    return ret

def get_neighbors(point: np.ndarray, previous: np.ndarray = None, next_points_only: bool = True):
    nearest_point = get_nearest_point(point)
    neighboring_points = midpoints.get(nearest_point, ())
    ret = []
    if next_points_only and previous is not None:
        previous_point = get_nearest_point(previous)
        for points in neighboring_points:
            temp = []
            for pt in points:
                if np.all(pt == previous_point):
                    break
                else:
                    temp.append(pt)
            if len(temp) < 2:
                temp = []
            for element in temp:
                ret.append(element)
    elif next_points_only and previous is None:
        print("Error! Cannot get next point neighbors without previous point")
    else:
        for points in neighboring_points:
            for pt in points:
                ret.append(pt)
    return ret

def plot_points(points):
    for point in points:
        plt.scatter(point[0], point[1], c='lime')

def plot(tri: Delaunay, points_cones, points, nearest_point, node: PathNode):

    # After performing Delaunay triangulation:
    #plt.triplot(points[:, 0], points[:, 1], tri.simplices, color='gray')

    # Plot the cones
    plot_points(points_cones)
    plt.scatter(nearest_point[0], nearest_point[1], c='r')
    path = np.array(node.extract_points([])).T
    if path.size > 0:
        #plt.plot(path[0], path[1], c='r')
        test=1
    plt.title('Path Planning')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.show()

class DelaunayPlanner(Node):
    def __init__(self):
        super().__init__('delaunay_planner')
        # Create a timer to run the planning routine once after startup
        self.position_data = POSITION
        self.cones = []
        #self.timer = self.create_timer(1.0, self.run_planner)

        # Subscribers
        self.create_subscription(
            PointStamped,
            'base_link_position',
            self.position_callback,
            10
        )

        self.create_subscription(
            PolygonStamped,
            'landmarks/filtered',
            self.cones_callback,
            10
        )

        self.publisher_midpoints = self.create_publisher(
            PolygonStamped,
            'delauney/midpoints',
            10
        )

        self.publisher_debug = self.create_publisher(
            Marker,
            'delauney/debug',
            10
        )

        self.get_logger().info('Delaunay Planner Node has been started.')

    def position_callback(self, msg: PointStamped):
        # Update position_data with the received data
        self.position_data = np.array([msg.point.x, msg.point.y])
        self.get_logger().info(f"Received new position: {self.position_data}")

    def cones_callback(self, msg: PolygonStamped):
        # Update cones with the received data
        self.get_logger().info(f"Received new cones data.")
        self.cones = msg.polygon.points
        self.cones.append(Point32(x=0.,y=1.5))
        self.cones.append(Point32(x=0.,y=-1.5))
        self.run_planner()

    def run_planner(self):
        global midpoints  # to allow the helper functions to access computed midpoints

        # Instead of reading CSV data, use hardcoded dummy data.
        points_dict = {
            "Blue Cones": [np.array([1, 1]), np.array([2, 2])],
            "Yellow Cones": [np.array([3, 1]), np.array([3, 3])],
            "Big Orange Cones": [np.array([1, 3])],
            "Unknown Cones": [np.array([2, 4])]
        }

        # Prepare points and Delaunay triangulation.
        #points = np.array([point for pts in points_dict.values() for point in pts])
        points = np.array([np.array([point.x, point.y]) for point in self.cones] + np.array([0,1.5])+ np.array([0,-1.5]))
        print(f'Before:  {points.shape}')
        distance_matrix = np.linalg.norm(points[:, None] - points, axis=2)
        np.fill_diagonal(distance_matrix, np.inf)
        neighbor_counts = np.sum(distance_matrix < 2.5, axis=1)
        flags = neighbor_counts > 2

        points = points[~flags]
        print(f'After : {points.shape}')
        tri = Delaunay(points)
        triangles = tri.points[tri.simplices]

        # Compute midpoints for each triangle.
        mid_list = []
        for triangle in triangles:
            mid_list.append(triple_midpoint(triangle[0], triangle[1], triangle[2]))
        midpoints = transform_structure(mid_list)

        debug : Marker = Marker()
        debug.header.frame_id = 'base_link'
        debug.header.stamp = self.get_clock().now().to_msg()
        debug.points = [Point(x = point[0], y = point[1]) for point in  midpoints]
        debug.action = 0
        debug.scale = Vector3(x = 0.5, y = 0.5, z = 0.5)
        debug.type = 8
        debug.color = ColorRGBA(r=0.,g=0.,b=1.,a=1.)

        # Compute the nearest point from POSITION.
        nearest_point = get_nearest_point(POSITION)
        if nearest_point is None:
            self.get_logger().error("Could not find a nearest point!")
            return

        # Initialize path planning.
        root_node = PathNode(np.array([-1, 0]))
        node = PathNode(np.array(nearest_point), root_node)
        node.expand(MAX_VIEWING_DISTANCE)
        print(node.options)

        calculated_midpoints  = node.extract_points([])
        output:PolygonStamped = PolygonStamped()
        output.header.frame_id = 'base_link'
        output.header.stamp = self.get_clock().now().to_msg()
        print(calculated_midpoints)
        output.polygon.points = [Point32(x = point[0], y = point[1]) for point in  calculated_midpoints]
        self.publisher_midpoints.publish(output)
        self.publisher_debug.publish(debug)
        # Plot the planning result.
        plot(tri, points, points, nearest_point, node)

        # Once done, cancel the timer.
        #self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = DelaunayPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
