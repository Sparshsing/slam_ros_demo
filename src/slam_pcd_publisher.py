#!/usr/bin/env python

import rospy
import pcl
from sensor_msgs.msg import PointCloud2

import struct
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

def open3d_to_ros(o3d_cloud, frame_id="frame1"):
    """
    Convert an Open3D tensor point cloud to a ROS PointCloud2 message
    o3d_cloud: Open3D tensor point cloud
    frame_id: Frame ID for the ROS message header
    """

    # Create ROS PointCloud2 message
    ros_cloud = PointCloud2()
    ros_cloud.header = Header(frame_id=frame_id)
    ros_cloud.height = 1  # Unordered point cloud

    # Assuming o3d_cloud has 'x', 'y', 'z', 'intensity', and 'ring'
    # points = o3d_cloud.point.positions.numpy()
    # intensities = o3d_cloud.point.intensity.numpy().reshape(-1, 1)
    # rings = o3d_cloud.point.ring.numpy().reshape(-1, 1)

    # # Combine x, y, z, intensity, and ring into a single array
    # combined = np.hstack((points, intensities, rings))

    dt = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32), ('ring', np.uint8)])
    structured_array = np.zeros(points.shape[0], dtype=dt)
    positions = o3d_cloud.point.positions.numpy()
    structured_array['x'] = positions[:,  0]
    structured_array['y'] = positions[:,  1]
    structured_array['z'] = positions[:,  2]
    structured_array['intensity'] = o3d_cloud.point.intensity.numpy().flatten()
    structured_array['ring'] = o3d_cloud.point.ring.numpy().flatten()

    # Define the fields for PointCloud2
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT8, count=1)
    ]

    # Create a buffer and fill it with the point data
    # buffer = []

    # for point in combined:
    #     buffer.append(struct.pack('ffffB', *point))

    # Update PointCloud2 message fields
    ros_cloud.fields = fields
    ros_cloud.is_bigendian = False  # Assuming little endian
    ros_cloud.point_step = 17  # Size of a point in bytes
    ros_cloud.row_step = ros_cloud.point_step * len(positions)
    ros_cloud.is_dense =  np.isfinite(positions).all() # int(np.isfinite(combined).all())
    # ros_cloud.data = b"".join(buffer)
    ros_cloud.data = structured_array.tobytes()

    return ros_cloud

def publish_pcd(pcd_file, publisher):
    # cloud = pcl.load(pcd_file)
    # ros_cloud = pcl_helper.pcl_to_ros(cloud)

    o3d_cloud = o3d.t.io.read_pointcloud(pcd_file)
    ros_cloud = open3d_to_ros(o3d_cloud)
    publisher.publish(ros_cloud)

def pcd_publisher():
    rospy.init_node('pcd_publisher_node', anonymous=True)
    pcd_topic = rospy.get_param('~pcd_topic', '/pcd')
    pcd_file_path = rospy.get_param('~pcd_file_path', 'path_to_your_pcd_file.pcd')
    publisher = rospy.Publisher(pcd_topic, PointCloud2, queue_size=10)
    rate = rospy.Rate(2)  # 1 Hz

    while not rospy.is_shutdown():
        publish_pcd(pcd_file_path, publisher)
        rate.sleep()

if __name__ == '__main__':
    try:
        pcd_publisher()
    except rospy.ROSInterruptException:
        pass
