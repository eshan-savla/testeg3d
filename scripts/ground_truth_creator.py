import numpy as np
import open3d as o3d


def main():
    ground_truth = o3d.geometry.PointCloud()
    y_dimension, x_dimension, z_dimension = 2000, 1000, 500
    points = np.zeros((y_dimension, x_dimension + z_dimension, 3), dtype=np.float64)
    start_val = 0.01
    step_size = 0.0001
    z_const_val = start_val
    x_const_val = start_val + ((x_dimension / 2) * step_size)
    z_top_val = start_val + (z_dimension * step_size)
    y_count = 0

    for line in points:
        y = start_val + (y_count * step_size)
        x_count, z_count = 0, 0
        for point in line:
            if x_count < x_dimension / 2:
                x = start_val + (x_count * step_size)
                z = z_const_val
                x_count += 1
            elif z_count >= z_dimension:
                x = start_val + (x_count * step_size)
                z = z_top_val
                x_count += 1
            else:
                x = x_const_val
                z = start_val + (z_count * step_size)
                z_count += 1
            point[0] = x
            point[1] = y
            point[2] = z
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
