# Резервная одометрия трамвая — материалы решения

C++17, ROS 2 Humble. Скорость и путь оцениваются по положению контроллера и двум датчикам тележек. Разрешённые GNSS-fix привязывают положение к карте маршрута. Поддержаны трамваи 30618 и 30639.

## Ссылки для формы загрузки

| Поле формы | Ссылка |
|---|---|
| Пакеты ROS 2 Humble | [Исходники `tram_odometry` и `tram_vehicle_msgs`](https://github.com/IGK-arch/Reserve-odometry-according-to-the-model/tree/main/ros2_ws) |
| Инструкция для жюри | [Сборка, rosbag, выходы, логи и измерения](https://github.com/IGK-arch/Reserve-odometry-according-to-the-model/blob/main/docs/JURY_CHECK.md) |
| Математическая модель | [Структура, уравнения, входы и выходы](https://github.com/IGK-arch/Reserve-odometry-according-to-the-model/blob/main/docs/submission/MODEL.md) |
| Допущения и конфигурируемые параметры | [Допущения, значения и способ настройки](https://github.com/IGK-arch/Reserve-odometry-according-to-the-model/blob/main/docs/submission/PARAMETERS.md) |
| Точность и быстродействие | [Методика, таблицы ошибок, график, задержки и ресурсы](https://github.com/IGK-arch/Reserve-odometry-according-to-the-model/blob/main/docs/submission/RESULTS.md) |
| Ограничения и развитие | [Границы применимости и план после хакатона](https://github.com/IGK-arch/Reserve-odometry-according-to-the-model/blob/main/docs/submission/LIMITATIONS.md) |

Проверенный рабочий код: `8164061`; описание соответствует конфигурации по умолчанию. Карта, профиль высоты, калибровочная таблица и типы сообщений входят в пакеты. Сырые rosbag для запуска предоставляет проверяющая сторона.
