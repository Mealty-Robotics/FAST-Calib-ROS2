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

LiDAR hole detection checks the full board cloud for empty interiors and distinct
centers. Four matching corners are preferred; `min_lidar_holes: 3` also accepts
three measured corners whose pair distances match the width, height and diagonal
(0.5, 0.4 and approximately 0.6403 m). `min_lidar_holes: 4` restores strict
four-corner operation. No missing center is synthesized: SVD uses only the three
or four observed centers in their original sensor coordinates.

Camera corner assignments are checked against pair distances and the known
mounting (+Z forward, +X down, +Y left in the raw LiDAR frame).
`max_mounting_rotation_deg` limits rotation away from that mounting (default
45 degrees, must be below 90) to resolve the rectangular target's symmetries.
This prior must agree with your mounting; it does not rotate the SVD inputs.

Tune the following parameters in `config/qr_params.yaml`; distances are meters:

| Parameter | Meaning |
| --- | --- |
| `circle_min_inliers` | Minimum points supporting each circle; separate from number of holes. |
| `circle_fit_distance` | RANSAC point-to-circumference tolerance. |
| `circle_radius_tolerance` | Radius/mean fit error tolerance and rim-support band. |
| `hole_geometry_tolerance` | Allowed side/diagonal and correspondence distance error. |
| `hole_max_interior_density` | Maximum inside/board density ratio; larger permits more interior returns. |
| `hole_max_interior_points` | Isolated interior returns allowed regardless of density. |
| `hole_min_surround_sectors` | Required surrounding board sectors out of eight. |
| `hole_max_angular_gap_deg` | Maximum unsupported rim angle; larger permits shorter arcs. |
| `boundary_search_radius` | Boundary neighborhood; zero selects automatic radius. |
| `edge_cluster_tolerance` | Edge clustering distance; zero selects automatic distance. |
| `min_points_per_voxel` | Accumulated return support; reduce for shorter/sparser recordings. |
| `plane_distance_threshold` | Board plane inlier distance. |

The interior disk uses 80% of `circle_radius`, compared with surrounding board
returns at 1.1–1.6 radii. The defaults allow two isolated interior returns or 5%
of board density, require six surrounding sectors and more than a semicircle
of rim support. Rim support uses the whole board, allowing split edge clusters.
Nearby fits are ranked by radius error before deduplication. Rejected fitted
inliers are removed and circle search continues.

If calibration fails, no new extrinsics are saved. The node keeps running until
Ctrl+C so RViz can show the scene, cropped board, boundaries, accepted candidate
centers (yellow), rejected circle centers (magenta), and rectified camera image.
The fixed frame is `calib_lidar`. Camera/LiDAR TF and projection overlay require
a valid calibration; a failed attempt labels the overlay image as image only.
The optional RViz **Camera view** display can be enabled after calibration succeeds.

Adjust the bag/image paths and XYZ crop for each recording. Keep the whole board
inside the crop. Detection thresholds do not automatically adapt to a particular
bag.

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
