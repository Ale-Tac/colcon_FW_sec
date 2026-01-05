#!/usr/bin/env python3
import sys
import yaml
import math
import rclpy
from rclpy.node import Node
from sensing_module.srv import PieceLocation
import numpy as np

SQUARE_SIZE = 0.05

def cell_to_xy(cell):
    file = cell[0].upper()
    rank = int(cell[1])
    file_idx = ord(file) - ord('A')
    rank_idx = rank - 1
    x = (file_idx - 3.5) * SQUARE_SIZE
    y = (rank_idx - 3.5) * SQUARE_SIZE
    return x, y

class CalibNode(Node):
    def __init__(self):
        super().__init__('calib_node')
        self.cli = self.create_client(PieceLocation, 'piece_location')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /piece_location service...')

    def query(self, aruco_id):
        req = PieceLocation.Request()
        req.aruco_id = int(aruco_id)
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done() and future.result() is not None:
            res = future.result()
            if res.found:
                return res.pose.position.x, res.pose.position.y
        return None


def compute_rigid_transform(A, B):
    # A and B are Nx2 arrays (measured -> expected), we find R,t minimizing ||R*A + t - B||
    assert A.shape == B.shape
    N = A.shape[0]
    centroid_A = A.mean(axis=0)
    centroid_B = B.mean(axis=0)
    AA = A - centroid_A
    BB = B - centroid_B
    H = AA.T @ BB
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1,:] *= -1
        R = Vt.T @ U.T
    t = centroid_B - R @ centroid_A
    angle = math.atan2(R[1,0], R[0,0])
    return angle, t[0], t[1], R, t


def main(argv):
    if len(argv) < 2:
        print('Usage: calibrate_sensing.py mapping.yaml')
        return 1

    mapping_file = argv[1]
    with open(mapping_file, 'r') as f:
        mapping = yaml.safe_load(f)

    rclpy.init()
    node = CalibNode()

    measured = []
    expected = []

    for aruco_id_str, cell in mapping.items():
        aruco_id = int(aruco_id_str)
        xy = node.query(aruco_id)
        if xy is None:
            node.get_logger().warn(f'arUco {aruco_id} not found by sensing')
            continue
        mx, my = xy
        ex, ey = cell_to_xy(cell)
        measured.append([mx, my])
        expected.append([ex, ey])
        node.get_logger().info(f'aruco {aruco_id}: measured=({mx:.4f},{my:.4f}) expected=({ex:.4f},{ey:.4f})')

    if len(measured) < 2:
        node.get_logger().error('Need at least 2 valid markers for calibration')
        return 2

    A = np.array(measured)
    B = np.array(expected)
    angle, tx, ty, R, t = compute_rigid_transform(A, B)

    print('\nCalibration result:')
    print(f'  rotation_rad: {angle}')
    print(f'  rotation_deg: {math.degrees(angle)}')
    print(f'  tx: {tx}')
    print(f'  ty: {ty}')

    # Print suggested C++ change snippet
    print('\nSuggested sensing transform to apply before pose_to_cell:')
    print('  x_new = cos(theta)*x - sin(theta)*y + tx')
    print('  y_new = sin(theta)*x + cos(theta)*y + ty')

    # Optionally write to file
    out = {'rotation_rad': float(angle), 'rotation_deg': float(math.degrees(angle)), 'tx': float(tx), 'ty': float(ty)}
    with open('/tmp/sensing_calibration.yaml', 'w') as f:
        yaml.safe_dump(out, f)
    node.get_logger().info('Wrote /tmp/sensing_calibration.yaml')

    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv))
