# Project LiDAR point to camera image

## Usage

### Docker

0. Configure launch file `launch/husky_lidar_projection.launch` or create your own, if needed. Do not forget to adjust last line in `docker/Dockerfile`: 

```
ENTRYPOINT [ "/ros_entrypoint.sh", "roslaunch", "/opt/ros/noetic/share/lidar_to_cam/launch/<your_launch_file>" ]
```

1. Build image:

```bash
bash docker/build.sh
```

2. Run node:

```bash
bash docker/run.sh
```
