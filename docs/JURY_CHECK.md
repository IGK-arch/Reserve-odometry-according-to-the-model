# Инструкция проверки решения

## 1. Сборка

Требуется Ubuntu 22.04, ROS 2 Humble, `colcon` и стандартные пакеты ROS, указанные в `package.xml`. Интернет во время сборки пакета не нужен. Из корня репозитория:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-up-to tram_odometry
source install/setup.bash
```

Команда собирает исходный пакет `tram_vehicle_msgs` и `tram_odometry`.

## 2. Запуск одного прогона

Терминал A:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
source install/setup.bash
ros2 launch tram_odometry tram_odometry.launch.py vehicle_id:=30618
```

Терминал B:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
source install/setup.bash
ros2 bag play /absolute/path/to/dataset/data/30618_2050d396 --clock
```

Для bag `30639_*` заменить параметр `vehicle_id:=30639`. Для отдельного прогона перезапустить ноду, чтобы её состояние начиналось заново. `--clock` удобен для наблюдения в ROS, но расчёт использует время из `header.stamp` и не зависит от частоты ROS clock.

## 3. Ожидаемые топики и проверка контракта

Терминал C с тем же ROS-окружением:

```bash
ros2 topic info /result/velocity -v
ros2 topic info /result/position -v
ros2 topic hz /result/velocity
ros2 topic hz /result/position
ros2 topic echo /result/diagnostics
```

Ожидаются:

| Топик | Тип | Поля для судьи |
| --- | --- | --- |
| `/result/velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | `header.stamp` и `velocity`, м/с |
| `/result/position` | `nav_msgs/msg/Odometry` | `header.stamp`, `header.frame_id`, `pose.pose.position.x/y/z`, `twist.twist.linear.x` |
| `/result/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | статусы сенсоров, slip, model-only, привязка, ускорение, задержка |

Частота двух обязательных результатов должна превышать 10 Гц при штатном проигрывании. В `header.stamp` должен стоять штамп исходного сообщения bag. Локальные скрипты сопоставляют его с GNSS-прокси в пределах 0,05 с; точный допуск судьи нам не сообщён. Во время движения с абсолютной привязкой `header.frame_id` у Odometry — `mgrs_37UCB`, при отсутствии начального GNSS и известного стартового места — `odom`. Диагностика содержит `callback_to_position_publish_ms` и `output_rate_hz_1s`; первое число измерено монотонными часами от входа в callback до завершения публикации позиции.

`/result/velocity` начинается сразу. `/result/position` начинается после начального GNSS-якоря; это исключает несколько ранних относительных точек `x≈0` из сравнения с MGRS `x≈100000`. Если GNSS нет, после `startup_gnss_window_s` (по умолчанию 5 с) начинается относительная одометрия. На коротком bag без GNSS можно выставить `use_startup_gnss: false` в копии конфигурационного YAML и передать её как `config:=/absolute/path/to/config.yaml`, чтобы относительное положение шло сразу. До выбора режима диагностика показывает `position_mode=pending_anchor`; старые точки задним числом не публикуются.

## 4. Проверка координат

Официальная система: UTM зона 37N с фиксированным началом MGRS-квадрата `37UCB`, `x=E−300000`, `y=N−6100000`. Значение `x` может быть больше 100000 на восточной части линии; скачка на границе 100-км квадратов быть не должно. Выход `/result/position` описывает `base_link`, точку оси передней тележки на уровне рельса. Начальный GNSS master установлен в `(-9.873,0,3.0)` м относительно `base_link`, rover в `(2.563,0,3.0)` м. Контроль организаторов: долгота 37.4602768500852°, широта 55.8088325462547° соответствует `x=103501.6309`, `y=85876.1201` м для самой точки GNSS.

При наличии только GNSS master стартовая ориентация оценивается по касательной карте. Если доступны обе антенны, направление продольной оси можно получить по вектору master→rover. После начальной выставки никакие GNSS данные не входят в фильтр движения. Карта поставляется в пакете и не строится на проверочном bag.

## 5. Воспроизведение локальных метрик без ROS

Нужен распакованный `dataset/data` и Python 3.12. Для `position_proxy.py` и построения графиков дополнительно нужны NumPy, SciPy, pyproj и Matplotlib; это **офлайн-инструменты анализа**, пакет ROS от них не зависит. На Windows или Linux (замените команду `py -3.12` на `python3` при необходимости):

```bash
python3 evaluation/core_benchmark.py --build --split validation --drive-table ros2_ws/src/tram_odometry/assets/drive_accel_table.csv --out evaluation/validation_core_table.csv
python3 evaluation/distance_proxy.py --split validation --drive-table ros2_ws/src/tram_odometry/assets/drive_accel_table.csv --out evaluation/validation_distance_table.csv
python3 evaluation/position_proxy.py --split validation --drive-table ros2_ws/src/tram_odometry/assets/drive_accel_table.csv --out evaluation/validation_position_table.csv
```

`core_benchmark.py` собирает CLI из **того же** `estimator.cpp`, что и ROS-нода. Параметр `--drive-table` повторяет режим ROS для трамвая `30618`; для `30639` таблица в ROS по умолчанию отключена. Replay идёт в порядке фактического поступления SQLite-сообщений, без будущих колесных отсчётов. Базовый метод использует среднюю доступную скорость тележек. GNSS и 3D-позиции читаются только отдельным кодом расчёта ошибок; полученные таблицы — прокси-оценка относительно сырых GNSS, а не скрытая метрика организаторов. Train/validation/holdout разделены в `tools/split_manifest.json` по полным сессиям и SHA-256 дубликатам.

## 6. Аномалии

В `/result/diagnostics` наблюдать `front_slip`, `rear_slip`, `front_stale`, `rear_stale` и `model_only`. На интервалах юза и буксования фильтр уменьшает вес подозрительного колеса; при потере обоих датчиков интегрирует ограниченное модельное ускорение и увеличивает ковариацию. После возврата согласованных колесных измерений заново привязывает скорость к ним. Для проверки поведения удобно открыть локальный 3D-просмотр из `analysis/simulator/README.md` и записи `30618_2050d396`, `30618_33bec73f`, `30639_50956d6e`.

## 7. Статус независимой проверки

В Ubuntu 22.04/ROS 2 Humble на WSL оба пакета успешно собраны через `colcon`. Реальные bag-прогоны для `30618`, `30639` и без GNSS прошли; полный движущийся bag `30618_af7496f0` за 263 с выдал 5313 сообщений скорости и 5312 положения при средней частоте 20,2 Гц. Метки времени и кадры проверены. JSON-отчёты и логи находятся в [`evaluation/ros_smoke`](../evaluation/ros_smoke), команда автоматического воспроизведения — `bash tools/ros_smoke_wsl.sh anchored` либо `no-gnss`. Измерение `callback_to_position_publish_ms` охватывает начало callback → вызов публикации внутри ноды; полную задержку от датчика через DDS и поведение на многочасовом прогоне эти измерения не подтверждают.

## 8. Сверка реальной ROS-ноды с автономным C++ replay

Повторный полный прогон `30618_af7496f0` с записью числовых выходов прошёл на ROS 2 Humble при скорости воспроизведения 1×. [Отчёт ноды](../evaluation/ros_smoke/anchored_20260926_233338_Uhja/report.json), [записанные выходы](../evaluation/ros_smoke/anchored_20260926_233338_Uhja/samples.csv) и [сравнение](../evaluation/ros_smoke/anchored_20260926_233338_Uhja/accuracy.json) сохранены вместе. На 5257 одинаковых метках RMSE скорости ROS относительно standalone CLI того же `estimator.cpp` равна 0,00273 м/с. ROS-скорость относительно доступного GNSS-прокси имеет RMSE 0,17481 м/с на 5307 метках; 3D-положение ROS относительно качественно отфильтрованной пары GNSS, приведённой к `base_link`, имеет RMSE 5,98 м на 5206 метках. Это локальные прокси-метрики одного holdout bag, а не скрытый score организаторов.

Для повтора из корня проекта в Ubuntu после сборки рабочего пространства:

```bash
export ROS_SMOKE_BAG="$PWD/dataset/data/30618_af7496f0"
export ROS_SMOKE_LOG_PARENT="$PWD/evaluation/ros_smoke"
export ROS_SMOKE_TIMEOUT_S=300
export ROS_SMOKE_RECORD=1
bash tools/ros_smoke_wsl.sh anchored
python3 evaluation/core_benchmark.py --build 30618_af7496f0 \
  --drive-table ros2_ws/src/tram_odometry/assets/drive_accel_table.csv
python3 evaluation/compare_ros_replay.py 30618_af7496f0 \
  evaluation/ros_smoke/<созданный_каталог>/samples.csv
```

Скрипт печатает фактический путь каталога логов в конце replay; подставьте его вместо `<созданный_каталог>`. Для вычисления 3D-прокси нужны NumPy, SciPy и pyproj. Запись CSV включается только `ROS_SMOKE_RECORD=1`, в обычном прогоне монитор хранит агрегаты. Полную задержку от публикации входа до приёма выхода другим DDS-потребителем и многочасовую устойчивость этот 263-секундный прогон не измеряет.
