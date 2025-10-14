# Ejecutar con Gazebo

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
ros2 launch gazebo_turtelbot draw_number.launch.py
ros2 topic echo /cmd_vel
```
