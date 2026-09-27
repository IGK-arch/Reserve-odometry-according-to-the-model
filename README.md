# Резервная одометрия трамвая

Приватный репозиторий команды: [IGK-arch/Reserve-odometry-according-to-the-model](https://github.com/IGK-arch/Reserve-odometry-according-to-the-model). Доступ к нему предоставляется только приглашённым участникам.

Решение для кейса Московского транспорта: C++17-оцениватель продольной скорости и пути, общее ядро навигации, ROS 2 Humble-нода, офлайн-карта линии и воспроизводимая проверка на rosbag. Скорость и путь вычисляются **только** по контроллеру и двум тележкам. Разрешённые GNSS-fix задают начальную привязку и ограниченные последующие поправки положения на карте; они не меняют скорость и накопленный путь. GNSS-скорость и `/localization/kinematic_state` не входят в рабочий оцениватель: эталон читает отдельный evaluator/checker.

## Где что находится

**Для жюри:** [все материалы решения и шесть ссылок для формы сдачи](docs/submission/README.md).

Последние эксперименты, принятые изменения и отклонённые гипотезы:
[раунд 4](docs/ROUND4_EXPERIMENTS.md). Двухсостоянийный Kalman проверен отдельно;
в поставляемой версии сохранён прежний продольный оцениватель.

| Каталог | Содержимое |
| --- | --- |
| [`ros2_ws/src/tram_odometry`](ros2_ws/src/tram_odometry) | ROS 2 пакет, C++ ядро, нода, запуск, конфигурация и готовая карта |
| [`ros2_ws/src/tram_vehicle_msgs`](ros2_ws/src/tram_vehicle_msgs) | исходные типы сообщений из датасета |
| [`evaluation`](evaluation) | replay общего C++ ядра, независимый эталонный evaluator, GNSS-прокси и ROS runtime-монитор |
| [`tools`](tools) | разделение train/validation/holdout и построение карты |
| [`docs`](docs) | методика карты, модель и инструкция жюри |
| [`analysis/simulator`](analysis/simulator) | интерактивный 3D-просмотр rosbag и аномалий |

Сырые bag находятся в локальном `dataset/data/` и исключены из публикации репозитория. Готовая карта `assets/route_map.csv` построена офлайн из обучающих записей и поставляется с пакетом.
Для другой западной конечной приложена `assets/route_map_branch_a.csv`: в направлении `out` ядро может выбрать её по устойчивому окну разрешённых GNSS-fix. При известном маршрутном задании карту можно задать явно. `assets/official_elevation.csv` содержит только высоту `base_link` из предоставленного организаторами Pathgraph; геометрия обоих путей и конечных остаётся обучающей картой.

## Запуск в ROS 2 Humble

На Ubuntu 22.04 после установки ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-up-to tram_odometry
source install/setup.bash
ros2 launch tram_odometry tram_odometry.launch.py vehicle_id:=30618
```

В другом терминале с тем же окружением:

```bash
ros2 bag play /path/to/30618_2050d396 --clock
```

Нода публикует `/result/velocity` (`tram_vehicle_msgs/msg/VelocitySensor`, м/с), `/result/position` (`nav_msgs/msg/Odometry`) и `/result/diagnostics`. Скорость выходит после инициализации по принятому колёсному измерению, положение — после короткой начальной привязки GNSS либо после ограниченного ожидания при её отсутствии; ранние относительные координаты не смешиваются с MGRS. Метки результатов берутся из входных сообщений. Числа в исходных колесных топиках — **км/ч**, что подтверждено организаторами, несмотря на комментарий в `.msg`.

Полная инструкция по настройке и проверке находится в [`ros2_ws/src/tram_odometry/README.md`](ros2_ws/src/tram_odometry/README.md). В частности, `vehicle_id` выбирает калибровку одного из двух трамваев; если стартовый GNSS отсутствует, нода выдаёт относительное положение с `frame_id=odom`.

## Координаты

Официальный целевой кадр — плоские координаты MGRS в фиксированном квадрате `37UCB`, положение `base_link` (передняя тележка на высоте головки рельса). На этом маршруте `x = UTM(37N).easting − 300000`, `y = UTM(37N).northing − 6100000`. Это **не** операция `mod 100000`: `x` остаётся непрерывным при пересечении границы соседнего 100-км квадрата. Проверочный пример организаторов: `lat=55.8088325462547`, `lon=37.4602768500852` → `x=103501.6309`, `y=85876.1201` м. Смещения GNSS-антенн от `base_link`: master `(-9.873, 0, 3.0)` м, rover `(2.563, 0, 3.0)` м. Подробности преобразования и ограничений приведены в документации ROS-пакета.

Текущий раунд: [12 экспериментов, исправление геометрии и все регрессии](docs/ROUND4_EXPERIMENTS.md). Новые ответы организаторов о кольцах и депо учтены.

## Воспроизводимость результатов

`tools/split_manifest.json` делит исходные уникальные bag **целыми сессиями**: 42 train, 13 validation, 42 holdout. Дубликаты SQLite по SHA-256 остаются в одном наборе. Чистые участки GNSS train использованы для масштабов колёс, таблицы привода и карты. `replay_cli` проверяет только продольное ядро; `navigation_replay` использует тот же `Navigation`, что ROS-нода, включая разрешённые GNSS-fix. Оба обрабатывают входы в порядке получения. Эталонная объединённая локализация не передаётся ни одному replay.

```powershell
py -3.12 evaluation/core_benchmark.py --build --split validation --drive-table ros2_ws/src/tram_odometry/assets/drive_accel_table.csv --table-vehicle 30618 --out evaluation/validation_core_table.csv
py -3.12 evaluation/position_proxy.py --split validation --drive-table ros2_ws/src/tram_odometry/assets/drive_accel_table.csv --table-vehicle 30618 --out evaluation/validation_position_table.csv
```

Эти команды дают **исторический GNSS-прокси** для исходного набора; `position_proxy.py` не воспроизводит новые периодические поправки общего ядра. Для предоставленного отдельно bag с `/localization/kinematic_state` используйте `evaluation/reference_benchmark.py`, а для официального ROS-сопоставления — `tools/ros_reference_check.sh`. Их протоколы и полный replay описаны в [`docs/JURY_CHECK.md`](docs/JURY_CHECK.md). Итоговые измерения и ограничения фиксируются в [`docs/RESULTS.md`](docs/RESULTS.md); эксперименты с отказами, прогнозом и геометрией — в [`docs/ROUND2_IMPROVEMENTS.md`](docs/ROUND2_IMPROVEMENTS.md).

Переносимая сборка и тесты без ROS, из корня репозитория. Python-тестам стресс-сценариев нужны NumPy и SciPy: `python3 -m pip install -r evaluation/requirements.txt`. В проверочном Docker-образе они уже установлены; C++ нода от них не зависит.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
python3 -m unittest discover -s evaluation/tests -v
```

Она собирает `build/replay_cli` и `build/navigation_replay`. C++-тесты используют поставляемые карты и не требуют bag; тесты evaluator не требуют ROS. Полный replay требует соответствующий датасет.

Полная таблица изменений метрик: [METRICS_COMPARISON.md](docs/METRICS_COMPARISON.md).
Исторический аудит исходной версии сокомандника: [VALIDATION_AUDIT.md](docs/VALIDATION_AUDIT.md).
Измерения в этом аудите относятся к версии до двух раундов улучшений.

## Текущий статус среды

В `evaluation/ros_smoke` сохранены **исторические** WSL/ROS 2 Humble-прогоны версии до общего навигационного ядра и периодических поправок. Они не подтверждают текущие точность, задержку или расход ресурсов. Актуальные проверенные отчёты публикуются в [`docs/RESULTS.md`](docs/RESULTS.md); проверка изменения доверия к колёсам — в [`docs/HEALTHY_WHEEL_GAIN_2026-09-27.md`](docs/HEALTHY_WHEEL_GAIN_2026-09-27.md). Скрипты построения карты используют Python-библиотеки только офлайн; после установки ROS-зависимостей пакет собирается без обращения к интернету.
