import numpy as np
import open3d as o3d
import os
import random


def main():
    ground_truth = o3d.geometry.PointCloud()
    y_dimension, x_dimension, z_dimension = 0.2, 0.1, 0.05  # Dimensions of step in meters
    start_val = 0.01
    step_size = 0.00025
    shift_points = True
    amplitude = 1.5 # variate between 0 & 0.8 for test
    y_no_points, x_no_points, z_no_points = int(2*y_dimension/step_size), int(2*x_dimension/step_size), int(2*z_dimension/step_size)
    points = np.zeros((y_no_points, x_no_points + z_no_points, 3), dtype=np.float64)
    z_const_val = start_val
    x_const_val = start_val + ((x_no_points / 2) * step_size)
    z_top_val = start_val + (z_no_points * step_size)
    y_count = 0

    for line in points:
        y = start_val + (y_count * step_size)
        x_count, z_count = 0, 0
        for point in line:
            if x_count < x_no_points / 2:
                x = start_val + (x_count * step_size)
                z = z_const_val
                x_count += 1
            elif z_count >= z_no_points:
                x = start_val + (x_count * step_size)
                z = z_top_val
                x_count += 1
            else:
                x = x_const_val
                z = start_val + (z_count * step_size)
                z_count += 1
            
            x_rand, y_rand, z_rand = 0, 0, 0
            if shift_points:
                rand_seed = os.urandom(10)
                random.seed(rand_seed)
                x_rand_val = random.uniform(-1, 1)*step_size * amplitude
                y_rand_val = random.uniform(-1, 1)*step_size * amplitude
                z_rand_val = random.uniform(-1, 1)*step_size * amplitude
                x_rand = x_rand_val
                y_rand = y_rand_val
                z_rand = z_rand_val

            point[0] = x + x_rand
            point[1] = y + y_rand
            point[2] = z + z_rand
        y_count += 1

    np.save("/home/eshan/TestEG3D/src/testeg3d/data/ground_truth", points)
    print("Wrote .npy file to ../data/ground_truth.npy")
    ground_truth.points = o3d.utility.Vector3dVector(np.concatenate(points, axis=0))
    print("Created point cloud with: ")
    print(ground_truth)
    o3d.io.write_point_cloud("/home/eshan/TestEG3D/src/testeg3d/data/ground_truth.pcd", ground_truth)
    print(".pcd file created at ../data/ground_truth.pcd")
    return None


if __name__ == "__main__":
    main()
