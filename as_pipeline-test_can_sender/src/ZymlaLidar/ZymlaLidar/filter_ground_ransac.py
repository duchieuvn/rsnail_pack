import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
from .read_points import read_points, numpy_to_pointcloud# ros needs the dot!
import sensor_msgs_py.point_cloud2 as pc2
import scipy
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions.
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message

    Code source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0

    References:
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
        http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html

    """
    # In a PointCloud2 message, the point cloud is stored as an byte
    # array. In order to unpack it, we also include some parameters
    # which desribes the size of each individual point.
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.
    print(f"To point_cloud {points.shape}")
    print(points)
    data = points.astype(dtype).tobytes()

    # The fields specify what the bytes represents. The first 4 bytes
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyza')]

    # The PointCloud2 message also has a header which specifies which
    # coordinate frame it is represented in.
    header = std_msgs.Header(frame_id=parent_frame)

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 4), # Every point consists of three float32s.
        row_step=(itemsize * 4 * points.shape[0]),
        data=data
    )


def do_transform_cloud(msg, tf):
        t = np.eye(4)
        q = tf.transform.rotation
        x = tf.transform.translation
        t[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        t[:3, 3] = [x.x, x.y, x.z]

        points_msg = pc2.read_points_numpy(msg)
        points_map = np.ones((len(points_msg), 4))
        points_map[:, :3] = points_msg
        points_map = np.dot(t, points_map.T).T

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        pcl_map_header = msg.header
        pcl_map_header.frame_id = "/velodyne_corr"
        pcl_map = pc2.create_cloud(pcl_map_header, fields, point_map[:, :3])
        return pcl_map

class FilterGround(Node):
    def __init__(self):
        super().__init__("bla")
        self.my_subscriber = self.create_subscription(PointCloud2, '/livox/lidar/transformed', self.callback, 1)

        # Create a TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Wait for transform and apply it
        try:
            self.transform = self.tf_buffer.lookup_transform(
                "/base_link", "/livox_frame", self.get_clock().now(), timeout=rclpy.duration.Duration(seconds=5.0)
            )
            
        except Exception as e:
            self.get_logger().error(f"Transform failed: {e}")

        self.my_publisher = self.create_publisher(PointCloud2, "/new_cloud", 1)


    def callback(self, msg: PointCloud2):
        #transform_cloud = do_transform_cloud(msg, self.transform)
        grid = self.divideData2Grid(msg)
        #cones = self.groundRemoval(grid, 5, 20, 0.02, 0.01)
        #print(f"cones {cones}")

        #new_pc = numpy_to_pointcloud(cones, frame_id='velodyne') # rviz: "Fixed Frame: velodyne"
        #new_pc = point_cloud(cones, "base_link")
        new_pc = point_cloud(np.array(grid), "base_link")
        self.my_publisher.publish(new_pc)



    def divideData2Grid(self, msg: PointCloud2):
        data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        # Convert the point cloud to a numpy array
        points = np.array(list(data))#, dtype=np.float32)
        points = [list(i) for i in points]
        points = np.array(points)
        # Filter points with z>=1
        points_map = points[:,2]<1
        points = points[points_map]

        print(f"Processing {points.shape[0]} points")
        # Step 1: Compute polar coordinates (r, theta) for each point
        xy = points[:, :2]  # X and Y coordinates
        r = np.linalg.norm(xy, axis=1)  # Euclidean distance in X-Y plane
        #filter_radius = r<20

        #r = r[filter_radius]
        #xy = xy[filter_radius]
        #points = points[filter_radius]
        #r = np.log10(r)

        filter_negative_x = points[:,0] > 0
        r = r[filter_negative_x]
        xy = xy[filter_negative_x]
        points = points[filter_negative_x]
        print(f"min point coordinates {points[:,0].min()} {points[:,1].min()}")
        print(f"negative x coord {sum(points[:,0]<0)}")
        print(f"Processing {points.shape[0]} points")
        theta = np.arctan2(points[:, 1], points[:, 0])  # Azimuth angle

        # Step 2: Define radial and angular bins
        # Number of bins = #(classes) + 1
        num_radial_bins = 2  # Number of distance-based bins
        num_angular_bins = 2  # Number of angular sectors

        # Radial bins: logarithmic spacing (or custom spacing)
        r_min, r_max = r.min(), r.max()
        radial_edges = np.linspace(np.power(r_min, (1/5)), np.power(r_max, (1/5))+1e-6, num=num_radial_bins+1)
        #radial_edges = 10 ** radial_edges
        #radial_edges = np.linspace(r_min, r_max, num=num_radial_bins)
        # Angular bins: uniform spacing
        angular_edges = np.linspace(-np.pi, np.pi, num=num_angular_bins+1)

        # Step 3: Assign points to bins
        radial_indices = np.digitize(np.power(r, (1/5)), radial_edges) - 1
        angular_indices = np.digitize(theta, angular_edges) - 1

        print(f"radial indices unqiue{np.unique(radial_indices, return_counts=True)}")
        print(f"angular indices unique{np.unique(angular_indices, return_counts=True)}")
        print(f"radial_indices==angular_indices {radial_indices.shape==angular_indices.shape}")
        print(f"radial_indices {radial_indices.shape}")
        print(f"max angle {theta.max()} -- min angle {theta.min()}")

        #print(f"radial indices:{list(radial_indices)}")
        #print(f"angular indices:{list(angular_indices)}")
        # Step 4: Organize points into grid cells
        adaptive_grid = {}
        colors = np.linspace(0,1, 4)
        color_grid = []

        for i in range(num_radial_bins):
            for j in range(num_angular_bins):
                adaptive_grid[(i, j)] = []

        for idx, (r_idx, theta_idx) in enumerate(zip(radial_indices, angular_indices)):
            #if 0 <= r_idx < num_radial_bins and 0 <= theta_idx < num_angular_bins:
            key = (r_idx, theta_idx)
            if key == (0,0):
                color_grid.append(np.append(points[idx], colors[0]))
                adaptive_grid[(r_idx, theta_idx)].append(points[idx])
            elif key == (1,0):
                color_grid.append(np.append(points[idx], colors[1]))
                adaptive_grid[(r_idx, theta_idx)].append(points[idx])
            elif key == (0,1):
                color_grid.append(np.append(points[idx], colors[2]))
                adaptive_grid[(r_idx, theta_idx)].append(points[idx])
            elif key == (1,1):
                color_grid.append(np.append(points[idx], colors[3]))
                adaptive_grid[(r_idx, theta_idx)].append(points[idx])
        #print(f"length {adaptive_grid[(2,2)]}")
        print(f"Points after griderization: {sum([len(adaptive_grid[(r,t)]) for r in range(num_radial_bins) for t in range(num_angular_bins)])}")
        #for r in range(num_radial_bins):
        #    for t in range(num_angular_bins):
        #     print(f"({r},{t}) {len(adaptive_grid[(r,t)])}")
        for keys in adaptive_grid.keys():
            print(f"keys {keys}")
            print(f"shape {len(adaptive_grid[keys])}")
        print(f"shape adaptive grid {len(adaptive_grid.keys())}")
        #print(f"shape00 {adaptive_grid[(0,0)]}")
        print(f"radial edges {radial_edges}")
        print(f"angular edges {angular_edges}")
        return color_grid

    def groundRemoval(self, grid, n_iter: int, n_lpr: int, th_seeds: float, th_dist: float):

        def extractInitialSeeds(P, n_lpr, th_seeds):
            seeds = []
            P_sorted = P[np.argsort(P[:, 2])]
            lpr = np.average(P_sorted[:n_lpr][:, 2], axis=0)
            for p in P_sorted:
                if p[2] < lpr+th_seeds:
                    seeds.append(p)

            return np.array(seeds)

        def estimatePlane(P_g):
            s_mean = np.average(P_g, axis=0)
            cov = np.cov(P_g, rowvar=False)
            #print(f"cov nan: {cov}")
            if np.isnan(cov).any():
                return np.array([0,0,0])
            U, S, V = scipy.linalg.svd(cov, compute_uv=True)
            n = U[np.argsort(S)[0]]
            nonlocal d_plane
            d_plane = -1*s_mean@(n/np.linalg.norm(n))
            return n
        
        model = None
        d_plane = None

        def computeDistance(p):
            #print(f"{model_unit = }")
            #print(f"{p = }")
            return (model_unit.T@p)-d_plane

        output_cloud = []
        colors = np.linspace(0,2**32-1, 12)

        for key, bi in zip(grid.keys(), grid.values()):
            if len(bi) <= 2:
                continue
            bi_nd = np.array(bi)
            #print(f"type {type(bi_nd)}")
            print(f"shape {bi_nd.shape}")
            print(bi_nd)
            P_g = extractInitialSeeds(bi_nd, n_lpr, th_seeds)
            P_ng = None
            for i in range(n_iter):
                if len(P_g) <= 2:
                    break
                model = estimatePlane(P_g)
                if np.equal(model,np.array([0,0,0])).all() == True:
                    break
                model_unit = model/np.linalg.norm(model)
                P_ng = None
                P_g = None
                print(f"{model_unit = }")
                distances = np.apply_along_axis(computeDistance, 1, bi_nd)
                print(f"distances {distances}")
                P_ng = bi_nd[distances >= th_dist]
                P_g = bi_nd[distances < th_dist]

            #if P_ng is not None:
            #    if key == (0,0):
            #         output_cloud.extend([np.append(p, colors[0]) for p in P_ng])
            #    elif key == (1,0):
            #        output_cloud.extend([np.append(p, colors[1]) for p in P_ng])
            #    elif key == (2,0):
            #        output_cloud.extend([np.append(p, colors[2]) for p in P_ng])
            #    elif key == (0,1):
            #        output_cloud.extend([np.append(p, colors[3]) for p in P_ng])
            #    elif key == (1,1):
            #        output_cloud.extend([np.append(p, colors[4]) for p in P_ng])
            #    elif key == (2,1):
            #        output_cloud.extend([np.append(p, colors[5]) for p in P_ng])
            #    elif key == (0,2):
            #        output_cloud.extend([np.append(p, colors[6]) for p in P_ng])
            #    elif key == (1,2):
            #        output_cloud.extend([np.append(p, colors[7]) for p in P_ng])
            #    elif key == (2,2):
            #        output_cloud.extend([np.append(p, colors[8]) for p in P_ng])
            #    elif key == (0,3):
            #        output_cloud.extend([np.append(p, colors[9]) for p in P_ng])
            #    elif key == (1,3):
            #        output_cloud.extend([np.append(p, colors[10]) for p in P_ng])
            #    elif key == (2,3):
            #        output_cloud.extend([np.append(p, colors[11]) for p in P_ng])

                output_cloud.extend(P_ng)
        return np.array(output_cloud)



def main(args=None):
    # Step 1: Initialize the ROS 2 Python library
    rclpy.init(args=args)

    # Step 2: Create the node
    node = FilterGround()

    # Step 3: Spin the node to keep it running
    rclpy.spin(node)

    # Step 4: Cleanup when the node is stopped
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
