# 7-Terminals Setup for Mapping

## General Instructions
- 5 terminals on ssh on ExoMy
- If ROS 2 topics do not appear, disable the firewall:
  ```sh
  sudo ufw disable
  ```
- Check ROS 2 topics:
  ```sh
  ros2 topic list
  ```

---

## Steps

### Terminal 1 (ExoMy ssh)
Start the ExoMy Docker container:
```sh
cd ExoMy_Software/docker
sh run_exomy.sh -a
```

### Terminal 2 (ExoMy ssh)
After the docker is created you can start `joy_publisher`:
```sh
# Get the container name for ExoMy (should be exomy_autostart)
docker ps
docker exec -it exomy_autostart bash
source exomy_ws/install/setup.sh
```
Prepare this line of code but don't run it:
```sh
ros2 run joy_publisher joy_publisher
```

### Terminal 3 (ExoMy ssh)
Prepare `position_calculator`:
```sh
docker exec -it exomy_autostart bash
source exomy_ws/install/setup.sh
```
Prepare this line of code but don't run it:
```sh
ros2 run joy_publisher position_calculator
```

### Terminal 4 (ExoMy ssh)
Prepare `imu_publisher`:
```sh
# Replace <ID> with the actual container ID
docker exec -it <ID> bash
source exomy_ws/install/setup.sh
```
Prepare this line of code but don't run it:
```sh
ros2 run joy_publisher imu_publisher
```

### Terminal 5 (ExoMy ssh)
Start `tof_publisher`:
```sh
cd ExoMy_Sensors/
sh launch.sh
source install/setup.sh
```
Prepare this line of code but don't run it:
```sh
ros2 run raw_data_publisher tof_publisher
```


### Terminal 6 (Host PC, Not SSH)
Start `point_cloud_creator1`:
```sh
export ROS_DOMAIN_ID=7
cd Exomy_Groundstation/
colcon build
source install/setup.sh
```
Prepare this line of code but don't run it:
```sh
ros2 run depth_processor point_cloud_creator1  # (This will change with collaboration)
```

### Terminal 7 (Host PC)
Start `rviz2` visualization:
```sh
export ROS_DOMAIN_ID=7
rviz2
```
#### RViz Configuration:
1. Add `PointCloud2` -> Topic: `/point_cloud1` -> Color Transformation: `AxisColor`
2. Add `TF`
3. Rename `Fixed Frame` to `world`

---

# Execution Steps
1. **Position** -> Terminal 3
2. **IMU** -> Terminal 4
Once you see the TF reference frame of the imu appear in rviz2 continue
3. **ToF** -> Terminal 5
4. **Point Cloud** -> Terminal 6
Wait for Point Cloud to be published with imu reference frame, then:
5. **Joy Publisher** -> Terminal 7

## Stopping Processes to Free RAM
Press `CTRL+C` on the following processes:
- `imu_publisher`
- `tof_publisher`
- `position_calculator`
- `joy_publisher`

Once done, `CTRL+C` the `point_cloud_creator1` process, which will generate two figures.

---
## Collaboration

### On another ExoMy via ssh:
### Terminal 8 (ExoMy2 ssh)
Start `tof_publisher`:
```sh
cd ExoMy_Sensors/
sh launch.sh
source install/setup.sh
```
Prepare this line of code but don't run it:
```sh
ros2 run raw_data_publisher tof_publisher
```


### Terminal 9 (Host PC, Not SSH)
Start `point_cloud_creator2`:
```sh
export ROS_DOMAIN_ID=7
cd Exomy_Groundstation/
colcon build
source install/setup.sh
```
Prepare this line of code but don't run it:
```sh
ros2 run depth_processor point_cloud_creator2
```

### Terminal 10 (Host PC, Not SSH)
Start `global_map`:
```sh
export ROS_DOMAIN_ID=7
cd Exomy_Groundstation/
colcon build
source install/setup.sh
```
Prepare this line of code but don't run it:
```sh
ros2 run depth_processor global_map
```
#### RViz Configuration:
1. Add `PointCloud2` -> Topic: `/point_cloud2` -> Color Transformation: `AxisColor`
2. Add `PointCloud2` -> Topic: `/point_cloud2` -> Color Transformation: `AxisColor`
3. Ensure only `global map` `PointCloud2` is ticked for merged map viewing

# Execution Steps
1. **Position** -> Terminal 3
2. **IMU** -> Terminal 4
Once you see the TF reference frame of the imu appear in rviz2 continue
3. **ToF** -> Terminal 5
4. **ToF** -> Terminal 8
5. **Point Cloud1** -> Terminal 6
6. **Point Cloud2** -> Terminal 9
7. **Point Cloud global map** -> Terminal 10
Wait for Point Cloud of ``global_map`` to be published with imu reference frames, then:
8. **Joy Publisher** -> Terminal 7
