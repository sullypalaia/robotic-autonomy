# TurtleBot3 Simulation


## References
| Topic                             | Primary Link                                                      | Alt / Notes                                                          |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------- |
| **TurtleBot3 Gazebo** (launch files & worlds)  | [https://github.com/ROBOTIS-GIT/turtlebot3\_simulations/tree/humble/turtlebot3\_gazebo](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble/turtlebot3_gazebo)                                                                                         | Debian/Ubuntu users: `sudo apt install ros-humble-turtlebot3-gazebo` |
| **Simulation e‑Manual** (one‑page install guide)| [https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)                                                                                                                           |                                                                      |
| **Robot description & URDFs**     | [https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble/turtlebot3\_description](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble/turtlebot3_description)                                                                                                        | Debian/Ubuntu users: `ros-humble-turtlebot3-description`.            |
| **Joystick driver** (`joy_linux`) | [https://index.ros.org/p/joy\_linux/](https://index.ros.org/p/joy_linux/)  | Debian/Ubuntu users: `sudo apt install ros-humble-joy-linux`         |
| **Passing USB port to docker** | on Linux machines you add add `"runArgs": ["--device", "/dev/input/js0:/dev/input/js0"]` as an argument to `.devcontainer.json`. On Windows machines you can use [usbipd](https://github.com/dorssel/usbipd-win). |  |
| **Tele‑op Twist bridge** (`teleop_twist_joy`)  | [https://wiki.ros.org/teleop_twist_joy](https://wiki.ros.org/teleop_twist_joy)                                                                                                                                                                             | Debian/Ubuntu users: `sudo apt install ros-humble-teleop-twist-joy`  |
| **Live image viewer** (`rqt_image_view`)   | [https://wiki.ros.org/rqt_image_view](https://wiki.ros.org/rqt_image_view)                                                                                                                                                                                 | Debian/Ubuntu users: `sudo apt install ros-humble-rqt-image-view`    |
| **Point‑cloud display in RViz2**  | [https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)                                                                   | `rviz2` Should be already installed                                  |
| **Rosbag record/play**            | [https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) |                                                                      |

---

### Minimal Install Commands

On any Ubuntu‑based host **or** inside the VS Code dev‑container:

```bash
sudo apt update && sudo apt install \
  ros-humble-turtlebot3-gazebo \
  ros-humble-joy-linux \
  ros-humble-teleop-twist-joy \
  ros-humble-rqt-image-view
```

---

### What "Done" Looks Like
* **Gazebo** window with TurtleBot3 world running.
* Gamepad steers the robot via `/cmd_vel`.
* `rqt_image_view` shows a live RGB feed.
* RViz2 displays `/camera/depth/color/points` in color.
* One `.db3` rosbag >= 10 MB captured during a short drive.
* Able to play back the drive

**Note:** You can use the joystick with `turtlesim` as well, which might be a better place to start before moving to the Gazebo simulation. You can also control the TurtleBot3 using the keyboard with `teleop_twist_keyboard` as a starting point as well.
