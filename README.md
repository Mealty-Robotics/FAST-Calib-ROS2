# FAST-Calib (ROS 2)

FAST-Calib is an automatic target-based extrinsic calibration tool for LiDAR-camera systems (eg., [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)).

**Key highlights include:**

1. Support solid-state and mechanical LiDAR.
2. No need for any initial extrinsic parameters.
3. Achieve highly accurate calibration results **in just 2 seconds**.

**In short, it makes extrinsic calibration as simple as intrinsic calibration.**

📬 For further assistance or inquiries, please feel free to contact Chunran Zheng at zhengcr@connect.hku.hk.

<p align="center">
  <img src="./pics/calib.jpg" width="100%">
  <font color=#a0a0a0 size=2>Left: Example of circle extraction from Mid360 point cloud | Right: Point cloud colored with calibrated extrinsic.</font>
</p>

## 1. Prerequisites

PCL>=1.8, OpenCV>=4.0.

## 2. Run our examples

1. Prepare the static acquisition data in `calib_data` folder (see [sample data](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/Eq_k_4Mf_11Eggg4a5lbRzgBHwd0EivtCJd2ExtcNlu1FA?e=vjm4gH) from Mid360, Avia and Ouster):

- rosbag containing point cloud messages
- corresponding image

2. Run the calibration process:

```bash
ros2 launch fast_calib calib_launch.py
```

RViz opens with calibration topics and connected sensor frames. `Camera view`
uses the photograph and CameraInfo to render the LiDAR scene from the camera's
viewpoint. `Projection overlay` shows depth-colored LiDAR points over the
rectified photograph; compare hole outlines, board edges, and scene boundaries.
Toggle `Full LiDAR scene` to compare the camera view with and without points.
Camera and LiDAR hole centers are green and red. The overlay is also saved as
`projection_overlay.png` beside the calibration results. These previews appear
before the full colored scene finishes processing. To run without RViz:

```bash
ros2 launch fast_calib calib_launch.py rviz:=false
```

The overlay marks the camera-derived hole centers with green crosses and the
transformed LiDAR centers with red crosses; yellow arrows show their difference.
`center_residuals.csv` records each correspondence's errors in meters and
rectified-image pixels. ArUco corner reprojection RMSE is logged separately, so
the 3D center-fit RMSE is not mistaken for pixel alignment accuracy. Supply the
complete camera distortion model: `k1`, `k2`, `p1`, `p2`, and `k3` (default 0).

The LiDAR mounting uses +Z forward, +X down, +Y left. Center sorting temporarily
maps this to camera optical axes as `(-Y, X, Z)`, then restores `(Y, -X, Z)`.
SVD receives the original sensor coordinates and sorting leaves its input cloud
unchanged. Configure the point-cloud driver without an additional rotation.

Calibration also publishes `calib_lidar_standard` (+X forward, +Y left, +Z up)
as a child of `calib_lidar`, with the same origin. Select this frame as RViz's
Fixed Frame to view the scene in standard LiDAR coordinates. The TF tree is
`calib_camera_optical -> calib_lidar -> calib_lidar_standard`; this last edge
uses pitch -90 degrees because its parent is the raw frame.

`calib_result.txt` maps raw LiDAR coordinates to the camera. The additional
`calib_result_standard.txt` maps standard LiDAR coordinates to the camera,
using `T_camera_standard = T_camera_raw * T_raw_standard`. Use that file only
when the consumer transforms the cloud into standard coordinates, not when
it merely changes the cloud's frame name. Both extrinsics have the same
translation because the two LiDAR frames share an origin.

## 3. Run on your own sensor suite

1. Customize the calibration target in the image below.
2. Record data to ros2 bag.
3. if you are using livox ros driver2 make sure that output of your points in PointCloud2 format (edit the launch file of desired livox LiDAR and make sure that xfer_format = 0 )
4. Provide the instrinsic matrix in `qr_params.yaml`.
5. Set distance filter in `qr_params.yaml` for board point cloud (extra points are acceptable).
6. Calibrate now!

LiDAR hole detection checks the full board cloud for empty interiors and requires
four distinct centers matching `delta_width_circles` and `delta_height_circles`
(0.5 × 0.4 m in the supplied configuration). All four sides and both diagonals
must match within 3 cm. The interior check uses 80% of `circle_radius` and
compares return density with the surrounding board (an annulus at 1.1–1.6 radii).
It tolerates two isolated returns or an interior density below 5% of the board
density, so sparse noise in longer recordings does not hide actual holes.
Surrounding board support must cover at least six of eight angular sectors.
Rim support across the full board must span more than a semicircle, rejecting
short exterior arcs while allowing separate edge clusters around one hole. Nearby fits are
ranked by error against the known radius before selecting distinct centers.
Rejected circle fits are removed
and the search continues; calibration exits with an error before estimating or
saving extrinsics if no valid four-hole target is found.

For accumulated scans, `min_points_per_voxel` removes weakly supported returns
before plane extraction (default 1; the current 128-scan bag uses 50).
`plane_distance_threshold` controls the board plane's inlier distance in meters
(default 0.01; the supplied configuration uses 0.02). Reduce voxel support for
shorter recordings. Boundary detection uses the aligned plane normal and a
neighborhood spanning adjacent scan rings.

The supplied crop, bag path, and image path are for the left-side recording and
`2.png`. Adjust the XYZ bounds when the board moves; the whole four-hole pattern
must remain inside the crop.

<p align="center">
  <img src="./pics/calibration_target.jpg" width="100%">
  <font color=#a0a0a0 size=2>Left: Actual calibration target | Right: Technical drawing with annotated dimensions.</font>
</p>

## 4. Appendix

Related article is coming soon...

The calibration target design is based on the [velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration).

For further details on the algorithm workflow, see [this document](https://github.com/xuankuzcr/FAST-Calib/blob/main/workflow.md).

## 5. Acknowledgments

Special thanks to [Jiaming Xu](https://github.com/Xujiaming1) for his support, [Haotian Li](https://github.com/luo-xue) for the equipment, and the [velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration) algorithm.
