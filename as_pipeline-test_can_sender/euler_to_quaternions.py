import math

def calculate_quaternion(z_deg, y_deg, x_deg):
    """
    Calculate the combined quaternion for sequential rotations around Z, Y, and X axes.

    Args:
        z_deg (float): Rotation around the Z-axis in degrees.
        y_deg (float): Rotation around the Y-axis in degrees.
        x_deg (float): Rotation around the X-axis in degrees.

    Returns:
        tuple: The resulting quaternion (qx, qy, qz, qw).
    """
    # Convert degrees to radians
    z_rad = math.radians(z_deg)
    y_rad = math.radians(y_deg)
    x_rad = math.radians(x_deg)
    
    # Compute individual quaternions for each axis
    qz = (0, 0, math.sin(z_rad / 2), math.cos(z_rad / 2))  # Z-axis rotation
    qy = (0, math.sin(y_rad / 2), 0, math.cos(y_rad / 2))  # Y-axis rotation
    qx = (math.sin(x_rad / 2), 0, 0, math.cos(x_rad / 2))  # X-axis rotation
    
    # Quaternion multiplication function
    def quaternion_multiply(q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        return (x, y, z, w)
    
    # Combine rotations: q_combined = qz * qy * qx
    q_combined = quaternion_multiply(qz, qy)
    q_combined = quaternion_multiply(q_combined, qx)
    
    return q_combined

