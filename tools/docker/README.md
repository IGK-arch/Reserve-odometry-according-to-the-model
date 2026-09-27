# ROS Humble в Docker

Проверено на Ubuntu 22.04/ROS 2 Humble arm64 в Docker. Контейнер нужен только для воспроизводимой Linux-проверки; сама нода собирается обычным colcon и не зависит от Docker/Python/SciPy во время работы.

```bash
docker build -t tram-odometry:humble -f tools/docker/Dockerfile .
docker run --rm --cpus 2 --memory 2g \
  -v "$PWD:/workspace" -w /workspace/ros2_ws tram-odometry:humble \
  bash -lc 'source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && colcon test && colcon test-result --verbose'
```

Официальный checker поставляется организаторами. После распаковки архива нужно собрать его `src/checker_ros` как отдельный Python ROS-пакет, используя **уже собранный** `tram_vehicle_msgs` решения. Не запускать демонстрационный `relay_result.py`: он копирует эталон в результаты и испортит проверку.

Например, поместить каталог `checker_ros` в `evaluation/runs/checker_ws/src/`, затем выполнить:

```bash
docker run --rm -v "$PWD:/workspace" -w /workspace tram-odometry:humble \
  bash -lc 'source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash && cd evaluation/runs/checker_ws && colcon build'

docker run --rm --cpus 2 --memory 512m -e ROS_DOMAIN_ID=89 \
  -e CHECKER_WS=/workspace/evaluation/runs/checker_ws \
  -v "$PWD:/workspace" \
  -v /absolute/path/to/30618_88aea4d9:/data/reference:ro \
  -w /workspace tram-odometry:humble \
  bash tools/ros_reference_check.sh --bag /data/reference \
    --outdir evaluation/runs/ros_reference \
    --config ros2_ws/src/tram_odometry/config/default.yaml \
    --ros-ws /workspace/ros2_ws --rate 1 --timeout 1800
```

Каждый запуск использует новый каталог результатов. В другом ROS_DOMAIN_ID можно параллельно проверять другую версию; не смешивать две ноды на одних `/result/*`. Для release-сборки без тестов допустим `-DBUILD_TESTING=OFF`. Производительность x86_64 и другие DDS-реализации нужно измерять отдельно.
