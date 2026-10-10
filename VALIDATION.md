# DOALS Hauptgebaeude sequence 1 validation

## Test bag

- Input: prepared DOALS Hauptgebaeude sequence 1
- Output: `data/doals_hauptgebaeude_sequence_1/sequence_1.bag`
- Duration: 107.8 s
- Messages: 1079 `sensor_msgs/PointCloud2` + 1079 `nav_msgs/Odometry`
- Point count: 131072 points per frame
- Compression: LZ4, 2.6 GiB (`rosbag info` displayed size)
- SHA-256: `c80d40ad4d063ec8e987bc0791a46d2e182651d3bbeed24d47c71abad4799baa`

The validation bag contains cloud and odometry messages with matching stamps
for deterministic ROS message-filter synchronization.

## Mapping smoke test

The first 10 seconds (about 100 frames) were replayed at 0.5x with the settings
in `config/doals_hauptgebaeude.yaml` and Dynablox filtering enabled.

Final published output:

- Allocated/known hash voxels: 97135
- Occupied voxels: 22379
- Occupancy probability range: [0.12, 0.97]
- ESDF range: [0.0, 4.749737] m (configured truncation: 5.0 m)
- Dynamic points in sampled publish frames: approximately 3059 to 8516
- `occupancy` and `esdf` fields: present and finite
- Validation result: PASS

The mapper library's ray-casting/ESDF and high-confidence ever-free dynamic
classification unit tests also pass. RViz 1.14.25 loaded
`rviz/doals_mapping.rviz` successfully with hardware OpenGL 4.6.

Reproduce:

```bash
roscore
roslaunch raycast_static_map doals_validation.launch rviz:=true
rosbag play $(rospack find raycast_static_map)/data/doals_hauptgebaeude_sequence_1/sequence_1.bag --rate 0.5
rosrun raycast_static_map check_mapping_output.py
```

