# Инструкция для жюри

Ubuntu 22.04, ROS 2 Humble, C++17, `colcon`. Команды выполняются из корня репозитория или распакованного решения. Параметры — поставляемый `default.yaml`.

## 1. Сборка

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to tram_odometry --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

На машине с установленными зависимостями шаг `rosdep` можно пропустить. Собираются `tram_vehicle_msgs` и `tram_odometry`. Карта и калибровки устанавливаются вместе с нодой.

## 2. Воспроизведение записи ROS (rosbag)

В каждом терминале откройте корень решения и выполните:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

**Терминал A — нода:**

```bash
ros2 launch tram_odometry tram_odometry.launch.py vehicle_id:=30618 2>&1 | tee node.log
```

**Терминал B — запись:** замените путь на каталог записи с `metadata.yaml`.

```bash
ros2 bag play /absolute/path/to/bag --clock --rate 1
```

Для трамвая 30639 укажите `vehicle_id:=30639`. Перед следующей записью остановите ноду через Ctrl+C и запустите заново. [Входы и единицы](submission/MODEL.md).

## 3. Ожидаемые выходы и логи

| Топик | Тип | Содержимое |
|---|---|---|
| `/result/velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | `velocity` в м/с. Точный `header.stamp` вызвавшего публикацию входа |
| `/result/position` | `nav_msgs/msg/Odometry` | Положение `base_link`: `pose.pose.position.x/y/z` в метрах. Скорость в `twist.twist.linear.x` |
| `/result/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Состояние колёс, привязка, ускорение, задержка и частота |

**Терминал C:**

```bash
ros2 topic info /result/velocity -v
ros2 topic info /result/position -v
ros2 topic hz /result/velocity
```

После измерения нажмите Ctrl+C и аналогично проверьте положение и диагностику:

```bash
ros2 topic hz /result/position
ros2 topic echo /result/diagnostics
```

Ожидаемая частота при штатной записи — выше 10 Гц. На полном контрольном прогоне около 21,75 Гц. Скорость появляется после первого принятого колёсного измерения. Положение — после стартовой GNSS-привязки. Без GNSS после окна 5 с выдаётся относительная одометрия. Результаты публикуются при поступлении новых входных сообщений.

| Проверка | Где смотреть |
|---|---|
| Абсолютная привязка | `frame_id=mgrs_37UCB`. Поля `position_mode`, `anchor_source` в диагностике |
| Относительная одометрия без GNSS | `frame_id=odom` |
| Работа по модели при потере колёс | `model_only`, `front_stale/rear_stale`, `front_slip/rear_slip` |
| Таблица ускорения загружена / используется | `drive_table_active`, `drive_table_used` |
| Задержка внутри обработчика сообщения | `callback_to_position_publish_ms`, мс |
| Частота публикации | `output_rate_hz_1s`, Гц |
| Ошибки запуска | Консоль терминала A и `node.log` |

Абсолютная система координат фиксирована: `x=UTM37N.E−300000`, `y=UTM37N.N−6100000`. Координата x может превышать 100000 м. Координаты сравниваются с эталоном напрямую.

## 4. Полный замер точности, задержки и ресурсов

Требуется собранный пакет `hackathon_solution_checker` в рабочем пространстве ROS и запись с `/localization/kinematic_state`. Для офлайн-пересчёта — `python3-numpy` и `python3-scipy`. Остановите ручной запуск из шага 2: следующая команда сама запускает ноду, проверяющую программу, проигрыватель и запись результатов.

```bash
source /opt/ros/humble/setup.bash
export CHECKER_WS=/absolute/path/to/built_checker_workspace
export ROS_DOMAIN_ID=87
bash tools/ros_reference_check.sh \
  --bag /absolute/path/to/reference_bag \
  --outdir evaluation/runs/jury_check \
  --config ros2_ws/src/tram_odometry/config/default.yaml \
  --ros-ws "$PWD/ros2_ws" --vehicle-id 30618 --rate 1 --timeout 1800
```

`CHECKER_WS/install/setup.bash` должен существовать. Каталог `--outdir` должен быть новым или пустым. Для записи длиннее 30 минут увеличьте `--timeout`. Эталон используется инструментами проверки точности.

| Файл в `evaluation/runs/jury_check/` | Содержимое |
|---|---|
| `run_summary.json` | `status=completed`, `playback_completed=true`, официальные метрики, задержки и CPU/RAM |
| `checker.log` | RMSE и максимальные ошибки официальной проверяющей программы |
| `node.log`, `player.log`, `recorder.log` | Логи процессов |
| `offline_metrics.json` | Дополнительный расчёт по ближайшей временной метке с допуском 50 мс |
| `result_bag/`, `candidate.csv` | Записанные результаты для анализа |
| `effective_parameters.yaml`, `runtime_samples.csv` | Конфигурация и замеры ресурсов |

[Методика и результаты](submission/RESULTS.md).

## 5. Проверка исходников без записи ROS

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
python3 -m unittest discover -s evaluation/tests -v
```

Проверены 19 C++ и 40 Python-тестов. Для Python-тестов нужны зависимости из `evaluation/requirements.txt`. [Настройка](submission/PARAMETERS.md), [ограничения](submission/LIMITATIONS.md).
