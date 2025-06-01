# BubbleBot

BubbleBot is an autonomous bubble tea preparation robot built on ROS 2.
It combines navigation, manipulator control, a recipe database, and visual workflow planning into a single integrated system.

## Features

* Autonomous navigation using TurtleBot3 and Nav2
* UART servo control for robotic arm operations
* SQLite database for recipes, ingredients, stations, and workflows
* Automatic generation of workflow steps based on orders
* Workflow diagram generation using Graphviz (`pydot`)
* PDF/PNG export for QR codes and task visualizations

## Project Structure

```
BubbleBot/
├── src/
│   ├── server_interface/         # ROS2 node for DB and workflow logic
│   ├── my_apriltag_pnp/          # AprilTag-based pose estimation
│   ├── my_servo_bridge/          # Serial bridge node for angle/status
│   └── ...
├── sql/
│   └── bubbletea.db              # SQLite database
├── outputs/                      # Auto-generated images
├── scripts/
│   └── draw_workflow.py          # Workflow visualization
├── README.md
```

## Getting Started

1. Clone into your ROS 2 workspace (e.g., `~/turtlebot_ws/src`)
2. Build and source the workspace:

   ```bash
   colcon build
   source install/setup.bash
   ```
3. Launch the ROS nodes:

   ```bash
   ros2 run server_interface ros_db_node
   ros2 run my_servo_bridge serial_bridge
   ```

## Visualize Workflow

Generate a PNG flowchart for a given order:

```bash
python3 draw_workflow.py --order_id 1 --mode horizontal
```

Available modes: `horizontal`, `vertical`, `doubleline`

## Dependencies

* ROS 2 Humble
* Python 3.8+
* Python packages:

  * `pydot`
  * `sqlite3` (built-in)
  * `reportlab` (for QR code layout export)

Install with:

```bash
pip install pydot reportlab
```

## License

MIT License

## Authors

* Huiyu Ren
* Yinuo Liu
* Weichao Zhou
* Richard Gan
* Thura Pyae Sone

