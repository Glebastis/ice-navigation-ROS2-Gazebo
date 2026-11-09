# ROS2-Gazebo: Навигация робота в снегу/на льду

## О проекте

Проект моделирует поведение мобильного робота на скользкой поверхности (снег, лёд), где возникает  
расхождение между показаниями одометрии и реальным положением робота.  

На текущем этапе реализовано **описание робота и его интеграция с Gazebo Classic**.  
Узел-возмутитель, контроллер и SLAM ещё не реализованы - см. раздел TODO.

## Стек

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic (Gazebo 11)

## Структура репозитория (на текущий момент)
```text
src/
└── bumperbot_description/
├── CMakeLists.txt
├── package.xml
├── launch/
│ ├── display.launch.py # RViz + robot_state_publisher (без симулятора)
│ └── gazebo.launch.py # Gazebo Classic + спавн робота
├── meshes/ # STL-меши звеньев робота
├── rviz/
│ └── display.rviz # Конфигурация RViz
└── urdf/
├── bumperbot.urdf.xacro
└── bumperbot_gazebo.xacro
```

## Что сделано

### 1. Описание робота (`bumperbot.urdf.xacro`)

- Дифференциальный привод: два ведущих колеса (`wheel_left_joint`, `wheel_right_joint`)
  и две пассивные опоры (`caster_front_link`, `caster_rear_link`).
- Звено `base_footprint` как точка отсчёта одометрии, `base_link` как корпус.
- IMU (`imu_link`) закреплён на корпусе.
- Инерциальные параметры (масса, тензор инерции) заданы для каждого звена.
- Коллизии колёс заданы сферами - осознанный компромисс для упрощения контактной задачи  
  в Gazebo Classic (см. раздел «Известные ограничения»).

### 2. Интеграция с Gazebo Classic (`bumperbot_gazebo.xacro`)

- **Дифференциальный привод**: плагин `libgazebo_ros_diff_drive.so`.  
  Публикует одометрию и TF `odom -> base_footprint`, подписан на `/cmd_vel_raw`.  
  Публикация одометрии и TF отключена (`publish_odom=false`, `publish_odom_tf=false`) -  
  одометрию будет считать отдельный узел контроллера.
- **Публикация состояний суставов**: плагин `libgazebo_ros_joint_state_publisher.so`  
  публикует `/joint_states` для обоих ведущих колёс с частотой 30 Гц.
- **IMU**: плагин `libgazebo_ros_imu_sensor.so` публикует `/imu/out`  
  с гауссовым шумом на угловые скорости и линейные ускорения.
- Параметры трения колёс настроены для «сухой» поверхности (`mu1=mu2=10.0`) -  
  это базовая точка, от которой в дальнейшем будет уменьшаться сцепление  
  для имитации снега/льда.

### 3. Launch-файлы

- `display.launch.py` - визуализация робота в RViz без симулятора.
  Запускает `robot_state_publisher` и `joint_state_publisher_gui`.
- `gazebo.launch.py` - запуск Gazebo Classic, спавн робота из `/robot_description`,
  установка `GAZEBO_MODEL_PATH`.

## Как запустить

1. Соберите workspace:
```bash
colcon build
source install/setup.bash
```

2. Проверить описание робота в RViz (без Gazebo):
```bash
ros2 launch bumperbot_description display.launch.py
```

3. Запустить Gazebo Classic со спавном робота:
```bash
ros2 launch bumperbot_description gazebo.launch.py
```

4. Проверить, что топики публикуются:
```bash
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /imu/out
```

На этом этапе робот появится в Gazebo, но не будет двигаться по командам -
для этого нужен bumperbot_controller (следующий коммит).

## Известные ограничения
- Коллизии колёс - сферы, а не цилиндры. Это упрощает контактную задачу, но приводит  
к небольшому «рысканию» при прямолинейном движении из-за численных погрешностей  
контактного решателя Gazebo Classic. Для задачи имитации скользкой поверхности  
это допустимо: часть «виляния» - артефакт симулятора, а не модели.  
- Gazebo Classic официально EOL с января 2025. Проект сознательно остаётся на нём  
из-за привязки к ROS 2 Humble и Ubuntu 22.04. Переход на Gazebo Sim (Ignition)  
запланирован как отдельная задача после завершения основного функционала.

## TODO

- [ ] bumperbot_controller: контроллер дифференциального привода (C++),  
считающий одометрию из /joint_states и публикующий TF odom -> base_footprint.
- [ ] bumperbot_controller: шумный контроллер с систематическими ошибками  
(радиус колеса, колея) и гауссовым шумом энкодеров.
- [ ] bumperbot_controller: узел-возмутитель (perturbation node) -  
см. описание принципиального отличия от шума в README следующего коммита.
- [ ] Демо-мир в Gazebo Classic (склад/снежная поверхность).
- [ ] SLAM (Graph SLAM / SLAM Toolbox) в условиях шумной одометрии.
- [ ] Переход на Gazebo Sim (Ignition) - отдельная задача, после стабилизации основного функционала.

## Disclaimer
- Описание робота и конфигурация симуляции в этом проекте основаны на учебных материалах  
Antonio Brandi (лицензия Apache 2.0).

- Оригинальные авторские права принадлежат Antonio Brandi. Контактное лицо для данного  
репозитория: Глеб Бабенко (babenkogleb@gmail.com).

Изменения относительно оригинальных материалов: адаптация плагинов под Gazebo Classic,  
отключение публикации одометрии из плагина diff_drive, переименование топика команд  
в /cmd_vel_raw, настройка параметров трения.

