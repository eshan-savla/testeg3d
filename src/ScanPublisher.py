#!/usr/bin/env python
import numpy as np
import open3d as o3d
import rospy
import os
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from tqdm import tqdm
from sensor_msgs.msg import PointCloud2
from testeg3d.msg import CloudData
from scipy.spatial.transform import Rotation as R

class PCLGenerator:
    def __init__(self, file, file_type:str, test: bool = False):
        self.filedata = np.load(file, allow_pickle=True)
        self.file_type = file_type
        self.test = test
        self.__is_exhausted = False
        self.save_pcl()

    def is_exhausted(self):
        return self.__is_exhausted 

    def get_scan_gap(self):
        scans = self.filedata[1]
        if(self.file_type == "old"):
            point1 = scans[0]["tfToWorld"][0]
            point2 = scans[1]["tfToWorld"][0]
        elif(self.file_type == "new"):
            point1 = scans[0]["tf_to_world"].asTuple()[0]
            point2 = scans[1]["tf_to_world"].asTuple()[0]
        gap = np.asarray(point2-point1)
        return np.linalg.norm(gap)


    def save_pcl(self):
        cwd = os.getcwd()
        # rospy.logdebug(f"Current directory: {cwd}")
        scans = self.filedata[1]
        points = []
        [points.append(scan['cloud']) for scan in scans]
        points = np.concatenate(points, axis=0)
        pcl = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        rospy.logwarn(f"Number of points in pointcloud: {len(pcl.points)}")
        o3d.io.write_point_cloud("/home/eshan/TestEG3D/src/testeg3d/data/before_filter.pcd", pcl)
        rospy.logdebug("Wrote unfiltered pointcloud file")

    def generate(self):
        scans = self.filedata[1]
        sum = 0
        for i in tqdm(range(len(scans)), desc="Scan progress"):
            # pos, ori = scan['tfToSensor']
            # T_toSensor = geomTupleToArray(pos, ori)
            if i >= (len(scans) - 1):
                self.__is_exhausted = True
            data = scans[i]
            if self.test and sum >= 11:
                break
            yield data
            sum += 1

    @staticmethod
    def on_shutdown():
        rospy.loginfo("Publisher shut down")

def arrayToPointcloud2(array, frame_id: str, timestamp=[]) -> PointCloud2:
    """Convert numpy array to PointCloud2

    Args:
        array (np.array): nx3 Array with xyz values
        frame_id (str): Frame name
        timestamp (rospy Time): Optional Timestamp

    Returns:
        PointCloud2: Converted pointcloud
    """

    header = Header(frame_id=frame_id)
    if timestamp:
        header.stamp = timestamp
    # create pcl from points
    pcl2 = pc2.create_cloud_xyz32(header, array)
    return pcl2

def publisher(file_type: str, test: bool = False):
    pub = rospy.Publisher('LaserValues', CloudData, queue_size=1000)
    rospy.loginfo("Created Publisher")
    rospy.init_node('laser_publisher', log_level=rospy.DEBUG)
    rospy.loginfo("Node initialized")
    rospy.on_shutdown(PCLGenerator.on_shutdown)
    rate = rospy.Rate(10)
    file = "/media/eshan/Eshans Stic/Scans/scan_230228-100944_fillet_inside.npy"
    virtual_pcl = PCLGenerator(file, file_type, test=test)
    generator = virtual_pcl.generate()
    i = 0
    first = True
    gap = virtual_pcl.get_scan_gap()
    while not rospy.is_shutdown() and not virtual_pcl.is_exhausted():
        pub_data = PointCloud2()
        data: np.ndarray = next(generator)
        points = data["cloud"]
        msg = CloudData()
        msg.gap = gap
        if(file_type == "old"):
            point_vector = data["tfToWorld"][0]
            quaternion = data["tfToWorld"][1]
        elif(file_type == "new"):
            point_vector = data["tf_to_world"].asTuple()[0]
            quaternion = data["tf_to_world"].asTuple()[1]
        msg.sensor_position = point_vector
        rot = R.from_quat(quaternion)
        vec_z = rot.apply(np.array([0, 0, 1]))
        msg.sensor_z = vec_z
        vec_y = rot.apply(np.array([0, 1, 0]))
        msg.sensor_y = vec_y
        vec_x = rot.apply(np.array([1, 0, 0]))
        msg.sensor_x = vec_x

        msg.first = False
        msg.last = False
        if first:
            msg.first = True
            first = False
        if virtual_pcl.is_exhausted():
            msg.last = True
        points_shape = points.shape
        first_points = points[0, :]
        msg.cloud = arrayToPointcloud2(points, "sensor", rospy.Time.now())
        # rospy.loginfo(f"Publishing data with first points: {first_points} and shape {points_shape}")
        rospy.logwarn_once(f"Publish started at: {rospy.get_time()}")
        pub.publish(msg)
        # rospy.loginfo("Published points from virtual pcl")
        rate.sleep()
    rospy.logwarn_once(f"Publish finished at: {rospy.get_time()}")
    



if __name__ == "__main__":
    try:
        publisher("new")
    except rospy.ROSInterruptException:
        rospy.logwarn("Kernel interrupted. Shutting publisher down")