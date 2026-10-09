#!/usr/bin/env python3
"""Validate the latched occupancy/ESDF outputs of raycast_static_map."""

import argparse
import math

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


def values(message):
    return list(point_cloud2.read_points(
        message, field_names=("occupancy", "esdf", "tsdf"), skip_nans=False))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("check_raycast_mapping_output", anonymous=True)
    known = rospy.wait_for_message("/raycast_static_map/known_voxels",
                                   PointCloud2, timeout=args.timeout)
    occupied = rospy.wait_for_message("/raycast_static_map/occupied_voxels",
                                      PointCloud2, timeout=args.timeout)
    required = {"x", "y", "z", "occupancy", "esdf", "tsdf"}
    fields = {field.name for field in known.fields}
    if not required.issubset(fields):
        raise RuntimeError(f"known_voxels is missing fields {required - fields}")
    data = values(known)
    if not data:
        raise RuntimeError("known_voxels is empty")
    if not all(math.isfinite(item[0]) and math.isfinite(item[1]) and
               math.isfinite(item[2]) for item in data):
        raise RuntimeError("occupancy, ESDF or TSDF contains non-finite values")
    occupancies = [item[0] for item in data]
    distances = [item[1] for item in data]
    tsdfs = [item[2] for item in data]
    print(f"known_voxels={known.width}")
    print(f"occupied_voxels={occupied.width}")
    print(f"occupancy_range=[{min(occupancies):.6f}, {max(occupancies):.6f}]")
    print(f"esdf_range=[{min(distances):.6f}, {max(distances):.6f}]")
    print(f"tsdf_range=[{min(tsdfs):.6f}, {max(tsdfs):.6f}]")
    print("mapping output validation: PASS")


if __name__ == "__main__":
    main()

