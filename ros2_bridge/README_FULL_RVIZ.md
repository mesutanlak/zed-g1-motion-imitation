# G1 tam ROS 2 / RViz görünümü

Bu katman yalnız gözlem ve görselleştirme içindir; fiziksel Unitree DDS komutu yayınlamaz.

## Başlatma

PowerShell'de proje klasöründen:

```powershell
powershell -ExecutionPolicy Bypass -File .\start_g1_full_rviz.ps1
```

ZED yakalama ve Isaac/GMR süreçleri ayrı pencerelerde çalışmaya devam eder. Başlatma sırası önemli değildir.

## Yayınlanan topicler

- `/zed/body38/markers`, `/zed/body38/keypoints`, `/zed/body38/local_orientations`
- `/zed/body38/root_pose`, `/zed/body38/keypoint_confidence`, `/zed/body38/diagnostics`
- `/g1/retarget/raw_joint_states`, `/g1/retarget/safe_joint_states`
- `/g1/isaac/joint_states`, `/g1/display/joint_states`
- `/g1/retarget/human_markers`, `/g1/retarget/raw_markers`, `/g1/retarget/safe_markers`
- `/g1/isaac/actual_markers`, `/g1/retarget/safety`, `/g1/retarget/diagnostics`
- `/g1/sensors/mount_markers`, `/robot_description`, `/tf`, `/tf_static`

Sensör kaynağı çalışırken RViz ayrıca `/livox/lidar`,
`/camera/camera/depth/color/points` ve
`/camera/camera/depth/image_rect_raw` topiclerini gösterir. Simülasyon sensörleri
`run_g1_zed_obstacles.sh --ros-sensors` yolundan; fiziksel sensörler ise resmi
Livox ROS Driver 2 ve RealSense ROS sürücüsünden gelir.

## Kontrol

```bash
export ROS_DOMAIN_ID=42
ros2 node list
ros2 topic list
ros2 topic hz /g1/display/joint_states
ros2 topic hz /zed/body38/keypoints
ros2 topic hz /livox/lidar
ros2 topic echo /g1/retarget/diagnostics --once
ros2 run tf2_ros tf2_echo odom pelvis
ros2 run tf2_ros tf2_echo torso_link mid360_link
ros2 run tf2_ros tf2_echo torso_link d435_link
```

`mid360_link` ve `d435_link` montajları Unitree'nin resmi `g1_23dof.urdf`
dosyasından gelir. ZED 2i burada robot üstü sensör değil, dış gözlem kamerasıdır;
`odom -> zed_camera` dönüşümü gerçek ekstrinsik kalibrasyonla daha sonra
değiştirilmelidir.
