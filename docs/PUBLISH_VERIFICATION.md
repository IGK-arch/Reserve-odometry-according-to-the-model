# Проверка объединённой версии перед публикацией — 27 сентября 2026

Объединены измеренная версия двух раундов `e248a96` и новый коммит сокомандника
`a9ad10c`. Сохранены общая реализация Navigation, исправления ковариации,
защита от повторных/старых заголовков команд и исторический аудит.
Новые правила ковариации используют текущий стартовый якорь Navigation;
масштаб линейных выходов согласован с масштабом их дисперсий.

| Проверка объединённого кода | Результат |
|---|---:|
| Portable Release CTest | 15/15 прошли |
| Повторная сборка распакованного исходного архива | 15 C++ / 37 Python прошли; runtime-файлы совпадают по SHA-256 |
| Python unittest | 37/37 прошли |
| Ubuntu 22.04 / ROS 2 Humble arm64 | 2 пакета собраны; 15/15 CTest прошли |
| Все уникальные записи, продольный replay до/после слияния | 97/97 файлов полностью совпали побайтно |
| Входы / выходы этой сверки | 3 999 054 / 2 061 664 строк |
| Открытый bag, полное Navigation | startup/corrections/branch/full: 4/4 CSV совпали побайтно |
| Выходов каждого public replay | 26 896 |
| Короткий ROS replay 30618 без GNSS | прошёл; frame odom; ковариации конечны |
| Короткий ROS replay 30639 с GNSS | прошёл; frame mgrs_37UCB; ковариации конечны |
| wheel_common_scale_sigma в ROS-диагностике | 0,01 в обоих прогонах |

Сверка сравнивает **полный CSV**, включая количество и метки выходов,
скорость, путь, флаги; public Navigation дополнительно включает XYZ и frame.
Это не только равенство округлённого RMSE. Защита команд проверена также
регрессионными тестами старых/повторных заголовков и сброса replay.
Тест опубликованной статистики проверяет ненулевой поворот, масштаб 0,5/2,
квадратичное изменение линейных дисперсий и неизменность угловых дисперсий.

Источники: [verification.json](../evaluation/results/publish_merge/verification.json),
[побайтная сверка и хэши](../evaluation/results/publish_merge/parity.json),
[portable CTest](../evaluation/results/publish_merge/portable_ctest.log),
[ROS CTest](../evaluation/results/publish_merge/ros_ctest.log),
[сборка и тесты распакованного архива](../evaluation/results/publish_merge/extracted_release_check.log),
[30618 без GNSS](../evaluation/results/publish_merge/ros_no_gnss.json),
[30639 с GNSS](../evaluation/results/publish_merge/ros_30639.json).

## Граница выводов

Полный 22-минутный ROS ATS-прогон после слияния **не повторялся**. Метрики
точности/ресурсов в [METRICS_COMPARISON.md](METRICS_COMPARISON.md) сохранены
с привязкой к измеренной версии `e248a96`. Совпадение детерминированных
выходов подтверждает отсутствие изменения оценок на проверенных записях,
но не является новым измерением DDS-задержки, ATS-пар или ресурсов.
Короткие ROS-проверки выполнены при 2 CPU / 512 MiB, таймаут каждого 25 с.
Все эти записи уже исследовались и не являются независимым скрытым тестом.

## Воспроизведение сверки

Сохранить сборку `e248a96` и собрать объединённый код отдельно. Входной public
CSV создаётся `evaluation/run_reference_ablation.py` из bag организаторов;
эталонная локализация в этот CSV не включается.

```sh
python3 evaluation/publish_parity.py \
  --before-build /path/to/e248a96-build \
  --after-build /path/to/merged-build \
  --public-input /path/to/permitted_inputs.csv \
  --out evaluation/runs/new_publish_parity
```

Исходные rosbags, context, бинарники и временные CSV в Git и исходный архив
не включены. Пакет воспроизводится `python3 tools/make_release.py`, проверяется
`python3 tools/make_release.py --verify-only`; manifest содержит SHA-256 файлов.
