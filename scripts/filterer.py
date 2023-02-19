import numpy as np
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm
import open3d as o3d

file = "/home/eshan/TestEG3D/src/testeg3d/data/scan_211115_163654.npy"
filedata = np.load(file, allow_pickle=True)
print("Loaded file")
write_filedata = filedata
scans = filedata[1]
write_scans = write_filedata[1]
size = len(scans)
points = []
[points.append(scan['cloud']) for scan in scans]
points = np.concatenate(points, axis=0)
pcl = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
downpcd = pcl.voxel_down_sample(voxel_size = 0.0001)
cl, ind = downpcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)
filterd_points = np.asarray(cl.points)
filterd_points.reshape(-1, -1, 3)
previous_size = 0
for i in tqdm(range(size), desc="Filtering Progress: "):
    data = scans[i]
    write_data = write_scans[i]
    point_vector = data["tfToWorld"][0]
    quaternion = data["tfToWorld"][1]
    rot = R.from_quat(quaternion)
    # vec_z = new_rot.apply(np.array([0, 0, 1]))
    # vec_y = new_rot.apply(np.array([0, 1, 0]))
    # vec_x = new_rot.apply(np.array([1, 0, 0]))
    points = data["cloud"]
    points_write = np.zeros((len(points), 3))
    for j in range(len(points)):
        points_write[j] = filterd_points[previous_size + j]
    previous_size += len(points)
    print(previous_size)
    tfToWorld_data = (point_vector, quaternion)
    write_data["tfToWorld"] = tfToWorld_data
    write_data["cloud"] = points_write
    write_scans[i] = write_data
    b = 0

write_filedata[1] = write_scans
print("Writing transformed file")
np.save("/home/eshan/TestEG3D/src/testeg3d/data/scan_211115_163654_filtered.npy",write_filedata, allow_pickle=True)
filename = "scan_211115_163654_filtered.npy"
print("Wrote file to %s" %filename)
input("Press enter to exit ....")