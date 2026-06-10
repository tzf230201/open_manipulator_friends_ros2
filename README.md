# Open_manipulator_friends

ROS 2 Humble status in this workspace:

- `open_manipulator_6dof_description` and `open_manipulator_6dof_moveit` are ament packages and build successfully.
- ROS 2 launch entry points are provided as `.launch.py` files.
- The C++ controller, teleop, and low-level libraries have been ported to `rclcpp`/ament, but they require ROS 2 packages that are not installed in this workspace: `open_manipulator_msgs`, `robotis_manipulator`, and `dynamixel_workbench_toolbox`.
- The legacy top-level `src/open_manipulator_friends` copy is ignored with `COLCON_IGNORE` so this ROS 2 tree owns the package names.

Validated locally:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select open_manipulator_6dof_description open_manipulator_6dof_moveit
source install/setup.bash
xacro install/open_manipulator_6dof_description/share/open_manipulator_6dof_description/urdf/open_manipulator_6dof.urdf.xacro > /tmp/open_manipulator_6dof.urdf
check_urdf /tmp/open_manipulator_6dof.urdf
ros2 launch open_manipulator_6dof_moveit demo.launch.py --show-args
```

After providing the missing Robotis/OpenManipulator ROS 2 dependencies, build the full port with:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  open_manipulator_6dof_libs \
  open_manipulator_6dof_controller \
  open_manipulator_6dof_teleop \
  open_manipulator_6dof_description \
  open_manipulator_6dof_moveit \
  open_manipulator_friends
```

## 1. Open_manipulator_6dof

### Related Video
- [OpenManipulator SARA Demonstration](https://www.youtube.com/watch?v=FexHPbmjwTc)

### Reference
- [3D Modeling (onshape)](https://cad.onshape.com/documents/e390b518d96db319b1ad08ef/w/a52f4418138084cfcd15bd6c/e/ba5c09be2e9d0922c49b7100?renderMode=0&uiState=6298082e5e421a4802d8c79c)
- [open_manipulation_6dof_application](https://github.com/zang09/open_manipulator_6dof_application.git)
- [open_manipulation_6dof_simulations](https://github.com/zang09/open_manipulator_6dof_simulations.git)
- [open_manipulation_perceptions](https://github.com/zang09/open_manipulator_perceptions.git)
