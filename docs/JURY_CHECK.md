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

Контракт требует частоту двух обязательных результатов выше 10 Гц при штатном проигрывании; её нужно измерять на конкретном прогоне. В `header.stamp` стоит точная метка вызвавшего публикацию входного сообщения. Предоставленный официальный checker использует `ApproximateTimeSynchronizer` с допуском 0,05 с и очередью 100, независимо для скорости и положения. Наш офлайн-evaluator использует другое правило: ближайший эталонный header в пределах включительно 50 мс, с возможным повторным использованием эталона. Эти способы нельзя объявлять эквивалентными. При абсолютной привязке `frame_id` у Odometry — `mgrs_37UCB`, в непривязанном режиме — `odom`. `callback_to_position_publish_ms` измеряет монотонными часами время от входа в callback до завершения вызова публикации; это не задержка всего тракта DDS.

`/result/velocity` начинается после инициализации по принятому колёсному измерению. `/result/position` начинается после начального GNSS-якоря; это исключает несколько ранних относительных точек `x≈0` из сравнения с MGRS `x≈100000`. Если GNSS нет, после `startup_gnss_window_s` (по умолчанию 5 с) начинается относительная одометрия. На коротком bag без GNSS можно выставить `use_startup_gnss: false` в копии конфигурационного YAML и передать её как `config:=/absolute/path/to/config.yaml`, чтобы относительное положение шло сразу после инициализации. До выбора режима диагностика показывает `position_mode=pending_anchor`; старые точки задним числом не публикуются.

## 4. Проверка координат

Официальная система: UTM зона 37N с фиксированным началом MGRS-квадрата `37UCB`, `x=E−300000`, `y=N−6100000`. Значение `x` может быть больше 100000 на восточной части линии; скачка на границе 100-км квадратов быть не должно. Выход `/result/position` описывает `base_link`, точку оси передней тележки на уровне рельса. Начальный GNSS master установлен в `(-9.873,0,3.0)` м относительно `base_link`, rover в `(2.563,0,3.0)` м. Контроль организаторов: долгота 37.4602768500852°, широта 55.8088325462547° соответствует `x=103501.6309`, `y=85876.1201` м для самой точки GNSS.

При наличии только GNSS master стартовая ориентация оценивается по касательной карте. Если доступны обе антенны, направление продольной оси можно получить по вектору master→rover. Разрешённые GNSS-fix после выставки могут корректировать продольный якорь карты и подтверждать выбор ветки; они не меняют скорость и пройденный путь `Estimator`. Скорость GNSS и `/localization/kinematic_state` нода не читает. Карта поставляется в пакете и не строится на проверочном bag.

Высота `base_link` на покрытом участке берётся из `assets/official_elevation.csv`, экспортированного из предоставленного организаторами Pathgraph, без чтения эталонных bag. Профиль меняет только высоту; оба рельсовых пути и конечные остаются обучающей картой. За пределами покрытия сохраняется высота карты. `elevation_file=none` отключает профиль, `alternate_map_file=none` — выбор другой ветки, `enable_gnss_corrections=false` — поздние поправки. Полное отключение GNSS: `use_startup_gnss=false`. Методика и ограничения — в [MAP_METHOD.md](MAP_METHOD.md).

## 5. Переносимые тесты и полный replay без ROS

Из корня репозитория, CMake 3.16+, компилятор C++17 и Python 3.10+:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
python3 -m unittest discover -s evaluation/tests -v
```

CMake собирает `build/navigation_replay` и `build/replay_cli`. Тринадцать C++
исполняемых наборов тестов, включая отказы колёс, ветвление и геометрию кузова,
используют поставляемые assets и не требуют bag. Python-тесты evaluator/runtime не требуют ROS.

Для отдельно предоставленного bag с объединённым эталоном:

```bash
python3 evaluation/reference_benchmark.py --bag /absolute/path/to/bag \
  --export-input /tmp/tram_inputs.csv
build/navigation_replay --input /tmp/tram_inputs.csv --output /tmp/tram_outputs.csv \
  --vehicle 30618 --gnss-mode corrections \
  --map ros2_ws/src/tram_odometry/assets/route_map.csv \
  --alternate-map ros2_ws/src/tram_odometry/assets/route_map_branch_a.csv \
  --elevation ros2_ws/src/tram_odometry/assets/official_elevation.csv \
  --drive-table ros2_ws/src/tram_odometry/assets/drive_accel_table.csv
python3 evaluation/reference_benchmark.py --bag /absolute/path/to/bag \
  --candidate /tmp/tram_outputs.csv --out /tmp/tram_metrics.json \
  --artifact build/navigation_replay \
  --artifact ros2_ws/src/tram_odometry/assets/route_map.csv \
  --artifact ros2_ws/src/tram_odometry/assets/route_map_branch_a.csv \
  --artifact ros2_ws/src/tram_odometry/assets/official_elevation.csv \
  --artifact ros2_ws/src/tram_odometry/assets/drive_accel_table.csv
```

Экспорт содержит только разрешённые C/F/R/MF/RF/MV/RV в порядке SQLite
получения. `navigation_replay` игнорирует GNSS-скорость и запрещает неизвестные
каналы. `/localization/kinematic_state` используется только последней командой
оценки. Replay и ROS вызывают общий `Navigation`; здесь пути assets заданы явно.
CLI не загружает ROS YAML: нестандартные ROS-параметры нельзя считать
автоматически воспроизведёнными. Для физики укажите `--table 0`; для режима
только начальной привязки — `--gnss-mode startup`, без GNSS — `off`.

JSON содержит RMSE/MAE/bias/max скорости по signed x, XYZ/3D-положение,
конечную ошибку, число выходов, invalid/unmatched, coverage, разрывы и хеши.
Baseline сравнивается на одинаковых метках с конечными значениями скорости
кандидата и эталона, при точной общей метке контроллера. Ближайший эталонный match не заменяет
официальную синхронизацию ROS.

### Исторический GNSS-прокси исходного набора

Нужен распакованный `dataset/data` и Python 3.12. Для `position_proxy.py` и построения графиков дополнительно нужны NumPy, SciPy, pyproj и Matplotlib; это **офлайн-инструменты анализа**, пакет ROS от них не зависит. На Windows или Linux (замените команду `py -3.12` на `python3` при необходимости):

```bash
python3 evaluation/core_benchmark.py --build --split validation --drive-table ros2_ws/src/tram_odometry/assets/drive_accel_table.csv --out evaluation/validation_core_table.csv
python3 evaluation/distance_proxy.py --split validation --drive-table ros2_ws/src/tram_odometry/assets/drive_accel_table.csv --out evaluation/validation_distance_table.csv
python3 evaluation/position_proxy.py --split validation --drive-table ros2_ws/src/tram_odometry/assets/drive_accel_table.csv --out evaluation/validation_position_table.csv
```

`core_benchmark.py` собирает CLI из **того же** `estimator.cpp`, что и ROS-нода, но не вызывает навигационный слой. Параметр `--drive-table` повторяет режим скорости ROS для `30618`; для `30639` таблица по умолчанию отключена. В продольный replay поступают только контроллер и колёса; GNSS читается отдельно для прокси-метрик. `position_proxy.py` сохраняется как прежний startup-only Python-прокси и не оценивает текущие поправки/ветвление/высоту `Navigation`. Train/validation/holdout разделены в `tools/split_manifest.json` по полным сессиям и SHA-256 дубликатам.

### Реальная ROS-нода и официальный checker

Нужен установленный пакет `hackathon_solution_checker` из предоставленного
организаторами checker workspace. После сборки решения, из корня репозитория:

```bash
CHECKER_WS=/absolute/path/to/checker_workspace ROS_DOMAIN_ID=87 \
  bash tools/ros_reference_check.sh \
  --ros-ws "$PWD/ros2_ws" \
  --bag /absolute/path/to/bag \
  --config "$PWD/ros2_ws/src/tram_odometry/config/default.yaml" \
  --vehicle-id 30618 --rate 1 \
  --outdir "$PWD/evaluation/runs/reference_run"
```

`--outdir` должен быть новым или пустым. Скрипт не запускает Docker сам:
его выполняют внутри готового ROS-окружения. Он ожидает discovery подписчиков,
возобновляет paused-player, записывает реальные выходы и reference отдельным
recorder, собирает финальный отчёт официального checker, затем независимо
считает офлайн-метрики. Нода не подписывается на reference. Таймаут,
неполный прогон, авария процесса или ошибка записи отмечаются как failed.

В каталоге остаются bag, `candidate.csv`, `offline_metrics.json`, логи,
`effective_parameters.yaml`, `runtime_samples.csv` и `run_summary.json`.
Последний сохраняет SHA-256 артефактов, официальные matched counts, статус
завершения, CPU процесса ноды относительно одного ядра, RSS и VmHWM. Runtime
наблюдатель отдельно сопоставляет вход/выход по точной header-метке и считает
монотонную wall receive-to-receive задержку p50/p95/max, unmatched и reorder.
Это измерение на стороне наблюдателя, не аппаратная задержка от датчика.
Recorder и checker имеют независимые DDS-подписки; их потери могут различаться.
Актуальные подтверждённые числа публикуются в [RESULTS.md](RESULTS.md).

## 6. Аномалии

В `/result/diagnostics` наблюдать `front_slip`, `rear_slip`, `front_stale`, `rear_stale`, `front_tentative`, `rear_tentative` и `model_only`. На интервалах юза и буксования фильтр уменьшает вес подозрительного колеса; при потере обоих датчиков интегрирует ограниченное модельное ускорение и увеличивает ковариацию. Согласие отвергнутой пары само по себе не снимает сильный скачок; восстановление только за счёт расширившейся неопределённости отмечается `tentative`. При исчезновении обоих здоровых каналов недавно выученный остаток ускорения кратко помогает прогнозу и затухает. Для проверки поведения удобно открыть локальный 3D-просмотр из `analysis/simulator/README.md` и записи `30618_2050d396`, `30618_33bec73f`, `30639_50956d6e`.

## 7. Историческая ROS-проверка прежней версии

Разделы 7–8 относятся к реализации до общего `Navigation`, периодических
поправок и официального профиля высоты. Они не подтверждают новые ROS-метрики.

В Ubuntu 22.04/ROS 2 Humble на WSL оба пакета успешно собраны через `colcon`. Реальные bag-прогоны для `30618`, `30639` и без GNSS прошли; полный движущийся bag `30618_af7496f0` за 263 с выдал 5313 сообщений скорости и 5312 положения при средней частоте 20,2 Гц. Метки времени и кадры проверены. JSON-отчёты и логи находятся в [`evaluation/ros_smoke`](../evaluation/ros_smoke), команда автоматического воспроизведения — `bash tools/ros_smoke_wsl.sh anchored` либо `no-gnss`. Измерение `callback_to_position_publish_ms` охватывает начало callback → вызов публикации внутри ноды; полную задержку от датчика через DDS и поведение на многочасовом прогоне эти измерения не подтверждают.

## 8. Историческая сверка ROS с продольным C++ replay

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
