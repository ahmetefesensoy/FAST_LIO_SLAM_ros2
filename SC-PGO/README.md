# SC-A-LOAM

## ROS 2 Humble: FAST-LIO + GNSS + RTK heading

Run the Scan Context pose-graph backend as `ros2 run aloam_velodyne sc_pgo`.
The legacy executable name `alaserPGO` remains installed as a compatibility
alias for older launch files; both names execute the same binary.

This branch keeps the original Scan Context/iSAM2 backend and adds a ROS 2
Humble integration for a FAST-LIO frontend.

Install the packaged GTSAM dependency and build:

```bash
sudo apt install ros-humble-gtsam
colcon build --packages-select aloam_velodyne
source install/setup.bash
ros2 launch aloam_velodyne pgo.launch.py
```

The pose-graph node consumes:

- `/cloud_registered_body` (`sensor_msgs/msg/PointCloud2`): deskewed keyframe cloud
- `/Odometry` (`nav_msgs/msg/Odometry`): high-rate FAST-LIO local odometry
- `/odometry/gps` (`nav_msgs/msg/Odometry`): ENU GNSS position and covariance
- `/rtk/heading_pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`): normalized ENU yaw and yaw variance in covariance index 35

The GNSS factor constrains X/Y only. The RTK factor constrains yaw only. Invalid
covariance, stale measurements, position innovations above the configured
Mahalanobis threshold, and yaw innovations above their threshold are rejected.
Set `heading_is_north_clockwise:=true` only when the input quaternion encodes a
north-zero, clockwise-positive vendor heading rather than normalized ENU yaw.
The conversion also supports antenna yaw offset and meridian convergence.

Measurements are associated by sensor timestamp. `measurement_max_past_age`
allows bounded delayed GNSS/RTK data, while `measurement_max_future_lead` is kept
small so a future observation is not attached to the wrong keyframe. The closest
candidate is selected; stale drops, future deferrals and last selected skew are
reported in `/pgo/diagnostics` and the final metrics JSON.
Set `antenna_offset_{x,y,z}` to the GNSS antenna origin expressed in
`base_frame`; the X/Y factor rotates this 3D lever arm with the estimated rover
attitude. With `initialize_from_absolute_measurements:=true`, the first valid
GNSS and heading pair establishes the graph's global X/Y/yaw datum instead of
being rejected against FAST-LIO's unrelated local origin.

The TF ownership is:

```text
map --(SC-PGO global correction)--> odom --(FAST-LIO local pose)--> base_link
```

SC-PGO computes `map -> odom` from each optimized keyframe and its matching
unmodified FAST-LIO keyframe. Disable `publish_map_to_odom` if another node owns
that transform. `map_frame`, `odom_frame`, `base_frame`, all input topics, and
all visualization output topics are parameters in `config/pgo_ros2.yaml`.

Run the focused tests with:

```bash
ctest --test-dir build/aloam_velodyne --output-on-failure
```

Before replaying a bag, validate its topic contract and obtain message counts
and average rates from `metadata.yaml`:

```bash
ros2 run aloam_velodyne bag_preflight.py /path/to/bag --profile rtk_fusion \
  --output /tmp/bag_preflight.json
```

Available contracts are `fastlio_raw`, `pose_graph`, and `rtk_fusion`; their
expected topic names and message types live in `config/bag_contract.yaml`.
Missing, empty, or wrong-type topics cause exit code 2, so the command can gate
automated bag benchmarks. This check does not claim localization accuracy; ATE,
RPE, CPU and memory require an actual recorded dataset and ground truth.

During replay, `/pgo/diagnostics` publishes keyframe and graph-update counts,
last iSAM2 update time in milliseconds, GNSS/heading accepted and rejected
counts, accepted loop count, and measurement queue depths. Record this topic in
benchmark bags so parameter sweeps can explain accuracy/runtime differences
instead of comparing trajectories alone.

Run the reproducible keyframe sweep after preflight succeeds:

```bash
ros2 run aloam_velodyne run_bag_benchmark.py /path/to/pgo_input_bag \
  --output /tmp/pgo_benchmark
```

The runner executes isolated `0.5`, `1.0`, and `2.0` metre keyframe runs,
records optimized odometry, diagnostics and TF, samples PGO CPU/RSS, and writes
`benchmark_report.json` plus `benchmark_summary.csv`. Use `--dry-run` to inspect
the exact commands without starting ROS processes. Each PGO process also writes
its final factor counters to `pgo_metrics.json`. ATE/RPE stays explicitly marked
`not_computed` until a ground-truth trajectory is supplied; resource or factor
metrics are never presented as localization accuracy.

The backend must be the only source of global correction when enabled. FAST-LIO
continues to own the smooth local `odom -> base_link` transform while SC-PGO
publishes only `map -> odom`.

## What is SC-A-LOAM? 
- A real-time LiDAR SLAM package that integrates A-LOAM and ScanContext. 
    - **A-LOAM** for odometry (i.e., consecutive motion estimation)
    - **ScanContext** for coarse global localization that can deal with big drifts (i.e., place recognition as kidnapped robot problem without initial pose)
    - and iSAM2 of GTSAM is used for pose-graph optimization. 
- This package aims to show ScanContext's handy applicability. 
    - The only things a user should do is just to include `Scancontext.h`, call `makeAndSaveScancontextAndKeys` and `detectLoopClosureID`. 

## Features 
1.  A strong place recognition and loop closing 
    - We integrated ScanContext as a loop detector into A-LOAM, and ISAM2-based pose-graph optimization is followed. (see https://youtu.be/okML_zNadhY?t=313 to enjoy the drift-closing moment)
2. A modular implementation 
    - The only difference from A-LOAM is the addition of the `laserPosegraphOptimization.cpp` file. In the new file, we subscribe the point cloud topic and odometry topic (as a result of A-LOAM, published from `laserMapping.cpp`). That is, our implementation is generic to any front-end odometry methods. Thus, our pose-graph optimization module (i.e., `laserPosegraphOptimization.cpp`) can easily be integrated with any odometry algorithms such as non-LOAM family or even other sensors (e.g., visual odometry).  
    - <p align="center"><img src="picture/anypipe.png" width=800></p>
3. (optional) Altitude stabilization using consumer-level GPS  
    - To make a result more trustworthy, we supports GPS (consumer-level price, such as U-Blox EVK-7P)-based altitude stabilization. The LOAM family of methods are known to be susceptible to z-errors in outdoors. We used the robust loss for only the altitude term. For the details, see the variable `robustGPSNoise` in the `laserPosegraphOptimization.cpp` file. 

## Prerequisites (dependencies)
- We mainly depend on ROS, Ceres (for A-LOAM), and GTSAM (for pose-graph optimization). 
    - For the details to install the prerequisites, please follow the A-LOAM and LIO-SAM repositiory. 
- The below examples are done under ROS melodic (ubuntu 18) and GTSAM version 4.x. 

## How to use? 
- First, install the abovementioned dependencies, and follow below lines. 
```
    mkdir -p ~/catkin_scaloam_ws/src
    cd ~/catkin_scaloam_ws/src
    git clone https://github.com/gisbi-kim/SC-A-LOAM.git
    cd ../
    catkin_make
    source ~/catkin_scaloam_ws/devel/setup.bash
    roslaunch aloam_velodyne aloam_mulran.launch # for MulRan dataset setting 
```

## Example Results 

### Riverside 01, MulRan dataset 
- The MulRan dataset provides lidar scans (Ouster OS1-64, horizontally mounted, 10Hz) and consumer level gps (U-Blox EVK-7P, 4Hz) data.
    - About how to use (publishing data) data: see here https://github.com/irapkaist/file_player_mulran
- example videos on Riverside 01 sequence. 
    1. with consumer level GPS-based altitude stabilization: https://youtu.be/FwAVX5TVm04
    2. without the z stabilization: https://youtu.be/okML_zNadhY 
- example result:

<p align="center"><img src="picture/riverside01.png" width=800></p>

### KITTI 05 
- For KITTI (HDL-64 sensor), run using the command 
    ```
    roslaunch aloam_velodyne aloam_velodyne_HDL_64.launch # for KITTI dataset setting
    ```
- To publish KITTI scans, you can use mini-kitti publisher, a simple python script: https://github.com/gisbi-kim/mini-kitti-publisher
- example video (no GPS used here): https://youtu.be/hk3Xx8SKkv4
- example result: 

<p align="center"><img src="picture/kitti05.png" width=800></p>

### Indoor
- ScanContext also works at indoor environments (use smaller sc_max_radius value).
- example video: https://youtu.be/Uv6_BRmxJho
- example result: 
<p align="center"><img src="picture/indoor.png" width=800></p>

### For Livox LiDAR 
- Scan Context also works for Livox LiDAR data
    - In this example, Scan Context is integrated with FAST-LIO (https://github.com/hku-mars/FAST_LIO).
    - Note: No additional integration effort is required. A user just run seperately FAST-LIO node and SC-A-LOAM's posegraphoptimization.cpp node!
- example video (tutoial and results): https://youtu.be/Fw9S6D6HozA
- example result: 
    <p align="center"><img src="picture/scfastlio.png" width=600></p>

### For Navtech Radar 
- Scan Context also works for Navtech Radar data!
- For the details, please see 
    - https://github.com/gisbi-kim/navtech-radar-slam
        - used the pose-graph optimization node of this repository (SC-A-LOAM)
    - [example video](https://www.youtube.com/watch?v=avtIQ8fesgU&t=128s)

## Utilities

### Data saver and Map construction 
- Similar to the [SC-LIO-SAM's saver utility](https://github.com/gisbi-kim/SC-LIO-SAM#applications), we support pose and scan saver per keyframes. Using these saved data, the map (within ROI) can be constructed offline. See the `utils/python/makeMergedMap.py` and [this tutorial](https://youtu.be/jmR3DH_A4Co). 
- Below is the example results of MulRan dataset KAIST 03's merged map, visualized using CloudCompare ([download the map data here](https://www.dropbox.com/sh/96jrpx3x6hh316j/AACb07kGbocnQWMIpksmU6MQa?dl=0)).  

    <p align="center"><img src="picture/kaist-03-merged.png" width=800></p>

- A user also can remove dynamic points using these saved keyframe poses and scans. See [this tutorial](https://www.youtube.com/watch?v=UiYYrPMcIRU) and our [Removert project](https://github.com/irapkaist/removert).

## Acknowledgements
- Thanks to LOAM, A-LOAM, and LIO-SAM code authors. The major codes in this repository are borrowed from their efforts.

## Maintainer 
- please contact me through `paulgkim@kaist.ac.kr` 

## TODO
- Delayed RS loop closings 
- SLAM with multi-session localization 
- More examples on other datasets (KITTI, complex urban dataset, etc.)
