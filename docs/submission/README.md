# Резервная одометрия трамвая — материалы решения

C++17, ROS 2 Humble. Скорость и путь оцениваются по положению контроллера и двум датчикам тележек. GNSS-координаты привязывают положение к карте маршрута. Поддержаны трамваи 30618 и 30639.

## Ссылки для формы загрузки

| Поле формы | Ссылка |
|---|---|
| Пакеты ROS 2 Humble | [Исходники `tram_odometry` и `tram_vehicle_msgs`](https://github.com/IGK-arch/Reserve-odometry-according-to-the-model/tree/main/ros2_ws) |
| Инструкция для жюри | [Сборка, воспроизведение записей, выходы, логи и измерения](https://github.com/IGK-arch/Reserve-odometry-according-to-the-model/blob/main/docs/JURY_CHECK.md) |
| Математическая модель | [Структура, уравнения, входы и выходы](https://github.com/IGK-arch/Reserve-odometry-according-to-the-model/blob/main/docs/submission/MODEL.md) |
| Допущения и конфигурируемые параметры | [Допущения, значения и способ настройки](https://github.com/IGK-arch/Reserve-odometry-according-to-the-model/blob/main/docs/submission/PARAMETERS.md) |
| Точность и быстродействие | [Методика, таблицы ошибок, график, задержки и ресурсы](https://github.com/IGK-arch/Reserve-odometry-according-to-the-model/blob/main/docs/submission/RESULTS.md) |
| Ограничения и развитие | [Границы применимости и план после хакатона](https://github.com/IGK-arch/Reserve-odometry-according-to-the-model/blob/main/docs/submission/LIMITATIONS.md) |

Описание соответствует конфигурации по умолчанию. Официальный открытый checker и ресурсные измерения сделаны на коде `8164061`; для текущей версии отдельно проверены [коррекция по остановкам](../STOP_LANDMARKS_2026-09-27.md), [защита при совместном залипании колёс](../PAIR_FREEZE_BRAKE_2026-09-27.md) и [правило выбора ветви](../BRANCH_BEHAVIOR_2026-09-27.md). При начальном окне GNSS 1,5 с последний кандидат дал 3D RMSE **2,615 м по GNSS-прокси** на 13 validation bag; эта выборка уже изучалась при разработке. Сборка ROS 2 Humble, **21/21 CTest** и **40/40 Python-тестов** прошли. Официальный балл последней версии пока не измерен. Карта, профиль высоты, каталог остановок, калибровочная таблица и типы сообщений входят в пакеты.

GitHub-репозиторий закрыт: ссылки в таблице доступны только аккаунтам, которым выдан доступ. Если жюри не сможет открыть его, передайте исходный ZIP `release/tram_odometry_source.zip` через платформу хакатона либо дайте экспертам доступ к приватному репозиторию.
