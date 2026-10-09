#!/usr/bin/env python3
"""Convert lidar_occ_world's converted DOALS sequence into a ROS1 bag."""

import argparse
from pathlib import Path

import numpy as np
import rosbag
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="ring", offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1),
]


def interpolate_pose(timestamp_ns, pose_times, poses):
    right = int(np.searchsorted(pose_times, timestamp_ns, side="left"))
    if right == 0:
        if pose_times[0] - timestamp_ns > 50_000_000:
            raise ValueError("cloud is more than 50 ms before the first pose")
        return poses[0]
    if right == len(pose_times):
        if timestamp_ns - pose_times[-1] > 50_000_000:
            raise ValueError("cloud is more than 50 ms after the final pose")
        return poses[-1]
    left = right - 1
    alpha = float(timestamp_ns - pose_times[left]) / float(pose_times[right] - pose_times[left])
    translation = (1.0 - alpha) * poses[left, :3] + alpha * poses[right, :3]
    q0 = poses[left, 3:7]
    q1 = poses[right, 3:7]
    if np.dot(q0, q1) < 0.0:
        q1 = -q1
    quaternion = (1.0 - alpha) * q0 + alpha * q1
    quaternion /= np.linalg.norm(quaternion)
    return np.concatenate((translation, quaternion))


def make_cloud(points, stamp, frame_id):
    message = PointCloud2()
    message.header = Header(stamp=stamp, frame_id=frame_id)
    message.height = 1
    message.width = points.shape[0]
    message.fields = FIELDS
    message.is_bigendian = False
    message.point_step = 24
    message.row_step = message.point_step * message.width
    message.is_dense = bool(np.isfinite(points[:, :3]).all())
    message.data = points.tobytes(order="C")
    return message


def make_odom(pose, stamp, frame_id, child_frame_id):
    message = Odometry()
    message.header = Header(stamp=stamp, frame_id=frame_id)
    message.child_frame_id = child_frame_id
    message.pose.pose.position.x = pose[0]
    message.pose.pose.position.y = pose[1]
    message.pose.pose.position.z = pose[2]
    message.pose.pose.orientation.x = pose[3]
    message.pose.pose.orientation.y = pose[4]
    message.pose.pose.orientation.z = pose[5]
    message.pose.pose.orientation.w = pose[6]
    return message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("sequence", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--frames", type=int)
    parser.add_argument("--cloud-topic", default="/doals/lidar_points")
    parser.add_argument("--odom-topic", default="/doals/odom")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--lidar-frame", default="lidar")
    args = parser.parse_args()

    timestamp_rows = []
    with (args.sequence / "timestamps.txt").open() as stream:
        for line in stream:
            timestamp, relative_path = line.split()
            timestamp_rows.append((int(timestamp), relative_path))
    if args.frames is not None:
        if args.frames < 1:
            parser.error("--frames must be positive")
        timestamp_rows = timestamp_rows[: args.frames]

    pose_table = np.loadtxt(str(args.sequence / "poses.txt"), dtype=np.float64)
    pose_times = pose_table[:, 0].astype(np.int64)
    poses = pose_table[:, 1:8]
    args.output.parent.mkdir(parents=True, exist_ok=True)

    with rosbag.Bag(str(args.output), "w", compression=rosbag.Compression.LZ4) as bag:
        for index, (timestamp_ns, relative_path) in enumerate(timestamp_rows):
            points = np.fromfile(str(args.sequence / relative_path), dtype=np.float32)
            if points.size % 6 != 0:
                raise ValueError(f"{relative_path} does not contain Nx6 float32 data")
            points = points.reshape((-1, 6))
            pose = interpolate_pose(timestamp_ns, pose_times, poses)
            stamp = rospy.Time(secs=timestamp_ns // 1_000_000_000,
                               nsecs=timestamp_ns % 1_000_000_000)
            odom = make_odom(pose, stamp, args.map_frame, args.lidar_frame)
            cloud = make_cloud(points, stamp, args.lidar_frame)
            bag.write(args.odom_topic, odom, stamp)
            bag.write(args.cloud_topic, cloud, stamp)
            if (index + 1) % 100 == 0 or index + 1 == len(timestamp_rows):
                print(f"wrote {index + 1}/{len(timestamp_rows)} frames", flush=True)

    print(f"created {args.output}")


if __name__ == "__main__":
    main()

