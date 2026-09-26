# Чувствительность GNSS-прокси и режимы движения

Скрипт `evaluation/audit_reference.py` заново проигрывает C++ оцениватель на сырых bag. Три разрешённых входа идут в оцениватель в порядке SQLite; GNSS используется только здесь для подсчёта ошибки. Таблица привода включена лишь для 30618.

## Приёмники GNSS

`master_only` и `rover_only` используют каждый приёмник отдельно; `dual_agree` требует оба и разницу скоростей ≤0,3 м/с. `published_composite` использует согласованную пару, а при отсутствии одного приёмника — другой. Все варианты сопоставляются на одинаковых выходных метках внутри своего варианта.

| Split | Трамвай | Прокси | Bag | Метки | RMSE база → модель, м/с | Δ |
|---|---|---|---:|---:|---:|---:|
| holdout | 30618 | dual_agree | 13 | 199689 | 0.1097 → 0.0936 | -0.0161 |
| holdout | 30618 | master_only | 13 | 205551 | 0.2448 → 0.2388 | -0.0060 |
| holdout | 30618 | published_composite | 13 | 209747 | 0.1369 → 0.1241 | -0.0128 |
| holdout | 30618 | rover_only | 13 | 204837 | 0.1103 → 0.0945 | -0.0158 |
| holdout | 30639 | dual_agree | 9 | 198370 | 0.1064 → 0.1098 | +0.0035 |
| holdout | 30639 | master_only | 9 | 199708 | 0.1613 → 0.1640 | +0.0027 |
| holdout | 30639 | published_composite | 10 | 222021 | 0.1108 → 0.1145 | +0.0037 |
| holdout | 30639 | rover_only | 10 | 221103 | 0.1302 → 0.1336 | +0.0033 |
| holdout | all | dual_agree | 22 | 398059 | 0.1081 → 0.1020 | -0.0060 |
| holdout | all | master_only | 22 | 405259 | 0.2079 → 0.2054 | -0.0025 |
| holdout | all | published_composite | 23 | 431768 | 0.1241 → 0.1192 | -0.0049 |
| holdout | all | rover_only | 23 | 425940 | 0.1211 → 0.1164 | -0.0046 |
| validation | 30618 | dual_agree | 13 | 276600 | 0.0675 → 0.0345 | -0.0330 |
| validation | 30618 | master_only | 13 | 282499 | 0.0694 → 0.0383 | -0.0311 |
| validation | 30618 | published_composite | 13 | 286515 | 0.0672 → 0.0345 | -0.0327 |
| validation | 30618 | rover_only | 13 | 280700 | 0.0854 → 0.0627 | -0.0227 |
| validation | all | dual_agree | 13 | 276600 | 0.0675 → 0.0345 | -0.0330 |
| validation | all | master_only | 13 | 282499 | 0.0694 → 0.0383 | -0.0311 |
| validation | all | published_composite | 13 | 286515 | 0.0672 → 0.0345 | -0.0327 |
| validation | all | rover_only | 13 | 280700 | 0.0854 → 0.0627 | -0.0227 |

## Режимы по GNSS-прокси

Для диагностики классы определены по эталонной скорости: стоянка <0,5 м/с; ускорение/торможение — изменение скорости на интервале ±0,5 с соответственно >+0,15 или <−0,15 м/с. Это разметка оценки с будущими GNSS-данными, не вход модели. `core_slip_flag` и `wheel_disagreement_gt_0p5` — пересекающиеся подвыборки, не отдельные непересекающиеся классы.

| Split | Трамвай | Режим | Bag | Метки | RMSE база → модель, м/с | Δ |
|---|---|---|---:|---:|---:|---:|
| holdout | 30618 | stop_ref_lt_0p5 | 13 | 63275 | 0.1628 → 0.1591 | -0.0038 |
| holdout | 30618 | accel_ref_gt_0p15 | 11 | 51468 | 0.1448 → 0.1224 | -0.0223 |
| holdout | 30618 | brake_ref_lt_minus_0p15 | 11 | 51544 | 0.1467 → 0.1234 | -0.0233 |
| holdout | 30618 | steady_moving | 11 | 43262 | 0.0380 → 0.0405 | +0.0026 |
| holdout | 30618 | moving_unknown_slope | 7 | 198 | 0.0573 → 0.0395 | -0.0178 |
| holdout | 30618 | core_slip_flag | 1 | 4 | 0.1676 → 0.9896 | +0.8220 |
| holdout | 30618 | core_model_only | 1 | 6 | 0.1717 → 0.9024 | +0.7307 |
| holdout | 30618 | baseline_no_fresh_wheel | 10 | 490 | 0.2084 → 0.1195 | -0.0889 |
| holdout | 30639 | stop_ref_lt_0p5 | 10 | 52742 | 0.0736 → 0.0710 | -0.0026 |
| holdout | 30639 | accel_ref_gt_0p15 | 10 | 56425 | 0.1192 → 0.1292 | +0.0100 |
| holdout | 30639 | brake_ref_lt_minus_0p15 | 10 | 61792 | 0.1384 → 0.1323 | -0.0061 |
| holdout | 30639 | steady_moving | 10 | 48138 | 0.0780 → 0.0974 | +0.0194 |
| holdout | 30639 | moving_unknown_slope | 7 | 2924 | 0.2354 → 0.2379 | +0.0024 |
| holdout | 30639 | wheel_disagreement_gt_0p5 | 4 | 27 | 0.5094 → 0.0937 | -0.4157 |
| holdout | 30639 | core_slip_flag | 4 | 66 | 0.3706 → 0.0927 | -0.2779 |
| holdout | 30639 | core_model_only | 10 | 153 | 0.0804 → 0.0555 | -0.0249 |
| holdout | 30639 | baseline_no_fresh_wheel | 10 | 378 | 0.1261 → 0.0814 | -0.0448 |
| holdout | all | stop_ref_lt_0p5 | 23 | 116017 | 0.1301 → 0.1269 | -0.0032 |
| holdout | all | accel_ref_gt_0p15 | 21 | 107893 | 0.1320 → 0.1260 | -0.0060 |
| holdout | all | brake_ref_lt_minus_0p15 | 21 | 113336 | 0.1422 → 0.1283 | -0.0139 |
| holdout | all | steady_moving | 21 | 91400 | 0.0623 → 0.0760 | +0.0137 |
| holdout | all | moving_unknown_slope | 14 | 3122 | 0.2283 → 0.2304 | +0.0021 |
| holdout | all | wheel_disagreement_gt_0p5 | 4 | 27 | 0.5094 → 0.0937 | -0.4157 |
| holdout | all | core_slip_flag | 5 | 70 | 0.3621 → 0.2531 | -0.1090 |
| holdout | all | core_model_only | 11 | 159 | 0.0856 → 0.1836 | +0.0979 |
| holdout | all | baseline_no_fresh_wheel | 20 | 868 | 0.1773 → 0.1046 | -0.0727 |
| validation | 30618 | stop_ref_lt_0p5 | 13 | 90169 | 0.0290 → 0.0208 | -0.0082 |
| validation | 30618 | accel_ref_gt_0p15 | 12 | 68206 | 0.0906 → 0.0430 | -0.0476 |
| validation | 30618 | brake_ref_lt_minus_0p15 | 12 | 69764 | 0.0890 → 0.0386 | -0.0504 |
| validation | 30618 | steady_moving | 12 | 57994 | 0.0424 → 0.0352 | -0.0073 |
| validation | 30618 | moving_unknown_slope | 11 | 382 | 0.0847 → 0.0408 | -0.0439 |
| validation | 30618 | wheel_disagreement_gt_0p5 | 2 | 182 | 1.3064 → 0.4340 | -0.8723 |
| validation | 30618 | core_slip_flag | 2 | 198 | 1.2566 → 0.4342 | -0.8225 |
| validation | 30618 | core_model_only | 2 | 29 | 1.7013 → 0.5624 | -1.1390 |
| validation | 30618 | baseline_no_fresh_wheel | 12 | 383 | 0.1661 → 0.0496 | -0.1165 |
| validation | all | stop_ref_lt_0p5 | 13 | 90169 | 0.0290 → 0.0208 | -0.0082 |
| validation | all | accel_ref_gt_0p15 | 12 | 68206 | 0.0906 → 0.0430 | -0.0476 |
| validation | all | brake_ref_lt_minus_0p15 | 12 | 69764 | 0.0890 → 0.0386 | -0.0504 |
| validation | all | steady_moving | 12 | 57994 | 0.0424 → 0.0352 | -0.0073 |
| validation | all | moving_unknown_slope | 11 | 382 | 0.0847 → 0.0408 | -0.0439 |
| validation | all | wheel_disagreement_gt_0p5 | 2 | 182 | 1.3064 → 0.4340 | -0.8723 |
| validation | all | core_slip_flag | 2 | 198 | 1.2566 → 0.4342 | -0.8225 |
| validation | all | core_model_only | 2 | 29 | 1.7013 → 0.5624 | -1.1390 |
| validation | all | baseline_no_fresh_wheel | 12 | 383 | 0.1661 → 0.0496 | -0.1165 |

## Покрытие

- validation: 13 bag с GNSS, 286515/286877 (99.9%) общих выходов оценены; меток с одним приёмником 9915, с несогласованной парой >0,3 м/с 42.
- holdout: 23 bag с GNSS, 431768/450089 (95.9%) общих выходов оценены; меток с одним приёмником 33709, с несогласованной парой >0,3 м/с 686.

Эти метрики условны на наличии и согласованности GNSS. Число отсчётов не равно числу независимых испытаний; для переноса на новые даты нужны новые сессии.
