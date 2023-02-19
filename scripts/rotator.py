import numpy as np
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

file = "/home/eshan/TestEG3D/src/testeg3d/data/scan_211115_163654.npy"
filedata = np.load(file, allow_pickle=True)
print("Loaded file")
write_filedata = filedata
scans = filedata[1]
write_scans = write_filedata[1]
size = len(scans)
for i in tqdm(range(size), desc="Transformation Progress: "):
    data = scans[i]
    write_data = write_scans[i]
    rot_90 = R.from_euler('z', 270, degrees=True)
    point_vector = data["tfToWorld"][0]
    quaternion = data["tfToWorld"][1]
    rot = R.from_quat(quaternion)
    new_rot = rot * rot_90
    quaternion_write = new_rot.as_quat()
    # vec_z = new_rot.apply(np.array([0, 0, 1]))
    # vec_y = new_rot.apply(np.array([0, 1, 0]))
    # vec_x = new_rot.apply(np.array([1, 0, 0]))
    points = data["cloud"]
    points_write = np.zeros((len(points), 3))
    rotated_point_vector = rot_90.apply(point_vector)
    for j in range(len(points)):
        points_write[j] = rot_90.apply(points[j])
    tfToWorld_data = (rotated_point_vector, quaternion_write)
    write_data["tfToWorld"] = tfToWorld_data
    write_data["cloud"] = points_write
    write_scans[i] = write_data
    b = 0

write_filedata[1] = write_scans
print("Writing transformed file")
np.save("/home/eshan/TestEG3D/src/testeg3d/data/scan_211115_163654_transformed.npy",write_filedata, allow_pickle=True)
filename = "scan_211115_163654_transformed.npy"
print("Wrote file to %s" %filename)
input("Press enter to exit ....")