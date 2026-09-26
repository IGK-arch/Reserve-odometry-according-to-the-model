# Фиксированные wheel-only альтернативы

Все модели оценены на одних и тех же выходных метках C++ replay и доступного GNSS-прокси. `front_only`/`rear_only` используют train-only масштабы колёс и hold-last при возрасте >0,25 с. `guarded`: при разнице колёс ≤0,3 м/с усредняет, иначе выбирает колесо ближе к предыдущей оценке; при двух отсутствующих удерживает значение. Порог выбран заранее и не подбирался на validation/holdout.

| Split | Трамвай | Метод | Bag | Метки | RMSE, м/с | Bias, м/с |
|---|---|---|---:|---:|---:|---:|
| holdout | 30618 | deployed_core | 13 | 209747 | 0.1241 | -0.0083 |
| holdout | 30618 | front_only | 13 | 209747 | 0.1371 | -0.0077 |
| holdout | 30618 | guarded | 13 | 209747 | 0.1368 | -0.0079 |
| holdout | 30618 | raw_average | 13 | 209747 | 0.1369 | -0.0090 |
| holdout | 30618 | rear_only | 13 | 209747 | 0.1370 | -0.0080 |
| holdout | 30639 | deployed_core | 10 | 222021 | 0.1145 | +0.0420 |
| holdout | 30639 | front_only | 10 | 222021 | 0.1742 | +0.0412 |
| holdout | 30639 | guarded | 10 | 222021 | 0.1215 | +0.0460 |
| holdout | 30639 | raw_average | 10 | 222021 | 0.1108 | +0.0307 |
| holdout | 30639 | rear_only | 10 | 222021 | 0.6957 | -0.0204 |
| holdout | all | deployed_core | 23 | 431768 | 0.1192 | +0.0176 |
| holdout | all | front_only | 23 | 431768 | 0.1573 | +0.0175 |
| holdout | all | guarded | 23 | 431768 | 0.1292 | +0.0199 |
| holdout | all | raw_average | 23 | 431768 | 0.1241 | +0.0115 |
| holdout | all | rear_only | 23 | 431768 | 0.5079 | -0.0144 |
| validation | 30618 | deployed_core | 13 | 286515 | 0.0345 | -0.0079 |
| validation | 30618 | front_only | 13 | 286515 | 0.0630 | -0.0085 |
| validation | 30618 | guarded | 13 | 286515 | 0.0608 | -0.0085 |
| validation | 30618 | raw_average | 13 | 286515 | 0.0672 | -0.0096 |
| validation | 30618 | rear_only | 13 | 286515 | 0.0805 | -0.0085 |
| validation | all | deployed_core | 13 | 286515 | 0.0345 | -0.0079 |
| validation | all | front_only | 13 | 286515 | 0.0630 | -0.0085 |
| validation | all | guarded | 13 | 286515 | 0.0608 | -0.0085 |
| validation | all | raw_average | 13 | 286515 | 0.0672 | -0.0096 |
| validation | all | rear_only | 13 | 286515 | 0.0805 | -0.0085 |

Эти альтернативы оценивают ценность модели относительно более сильных простых правил. Выбор лучшего метода по holdout был бы утечкой; ни одна альтернатива здесь не переключена в production.
