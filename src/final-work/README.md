# Editing this README

When you're ready to make this README your own, just edit this file and use the handy template below (or feel free to structure it however you want - this is just a starting point!). Thanks to [makeareadme.com](https://www.makeareadme.com/) for this template.

## Suggestions for a good README

Every project is different, so consider which of these sections apply to yours. The sections used in the template are suggestions for most open source projects. Also keep in mind that while a README can be too long and detailed, too long is better than too short. If you think your README is too long, consider utilizing another form of documentation rather than cutting out information.

## Name
Choose a self-explaining name for your project.

## Description
Let people know what your project can do specifically. Provide context and add a link to any reference visitors might be unfamiliar with. A list of Features or a Background subsection can also be added here. If there are alternatives to your project, this is a good place to list differentiating factors.

## Badges
On some READMEs, you may see small images that convey metadata, such as whether or not all the tests are passing for the project. You can use Shields to add some to your README. Many services also have instructions for adding a badge.

## Visuals
Depending on what you are making, it can be a good idea to include screenshots or even a video (you'll frequently see GIFs rather than actual videos). Tools like ttygif can help, but check out Asciinema for a more sophisticated method.

## Installation
Within a particular ecosystem, there may be a common way of installing things, such as using Yarn, NuGet, or Homebrew. However, consider the possibility that whoever is reading your README is a novice and would like more guidance. Listing specific steps helps remove ambiguity and gets people to using your project as quickly as possible. If it only runs in a specific context like a particular programming language version or operating system or has dependencies that have to be installed manually, also add a Requirements subsection.

## Usage
Use examples liberally, and show the expected output if you can. It's helpful to have inline the smallest example of usage that you can demonstrate, while providing links to more sophisticated examples if they are too long to reasonably include in the README.

## Support
Tell people where they can go to for help. It can be any combination of an issue tracker, a chat room, an email address, etc.

## Roadmap
If you have ideas for releases in the future, it is a good idea to list them in the README.

## Contributing
State if you are open to contributions and what your requirements are for accepting them.

For people who want to make changes to your project, it's helpful to have some documentation on how to get started. Perhaps there is a script that they should run or some environment variables that they need to set. Make these steps explicit. These instructions could also be useful to your future self.

You can also document commands to lint the code or run tests. These steps help to ensure high code quality and reduce the likelihood that the changes inadvertently break something. Having instructions for running tests is especially helpful if it requires external setup, such as starting a Selenium server for testing in a browser.

## Authors and acknowledgment
Show your appreciation to those who have contributed to the project.

## License
For open source projects, say how it is licensed.

## Project status
If you have run out of energy or time for your project, put a note at the top of the README saying that development has slowed down or stopped completely. Someone may choose to fork your project or volunteer to step in as a maintainer or owner, allowing your project to keep going. You can also make an explicit request for maintainers.




# SOLUTION

## Packages needed
```
ln -s ~/data/all_introduction_to_ros2_packages/kinenikros2
ln -s ~/data/all_introduction_to_ros2_packages/robotiq_85_gripper
ln -s ~/data/all_introduction_to_ros2_packages/robotiq_85_gripper_server
ln -s ~/data/all_introduction_to_ros2_packages/aruco_ros
ln -s ~/data/all_introduction_to_ros2_packages/aruco_broadcaster
ln -s ~/data/all_introduction_to_ros2_packages/tablesens
ln -s ~/data/all_introduction_to_ros2_packages/chesslab_setup2
ln -s ~/data/all_introduction_to_ros2_packages/chesslab_setup2_interfaces
```

## To test planning module

- Terminal 1
```
cd .../colcon_wsFinalWork
colcon build --packages-select kinenikros2 planning_module
source install/setup.bash
ros2 run kinenikros2 kinenik_srv_server
```
- Terminal 2
```
cd .../colcon_wsFinalWork
source install/setup.bash
ros2 launch planning_module planning_node.launch.py
```
- Terminal 3
```
source install/setup.bash
cd .../colcon_wsFinalWork
source install/setup.bash
ros2 service call /plan_pick_place planning_module/srv/PlanPickPlace \
"{source_pose: {position: {x: 0.3, y: 0.0, z: 0.03}, orientation: {w: 1.0}},
  target_pose: {position: {x: 0.4, y: 0.1, z: 0.03}, orientation: {w: 1.0}}}"
```
wanted message:
success: true
message: "Trajectory planned successfully"
trajectory:
...

## To test sensing module

- Terminal 1:
```
cd ~/data/colcon_wsFinalWork
colcon build --base-paths src --packages-skip action_manager_module action_manager_module_interfaces
source install/setup.bash
ros2 launch sensing_module chesslab_sensing.launch.py
```
- Terminal 2:
```
source install/setup.bash
ros2 service call /piece_location sensing_module/srv/PieceLocation "{aruco_id: 316}"
```
## Just for ale
use this to build 

colcon build --base-paths src --packages-skip action_manager_module action_manager_module_interfaces

## To test action module
- Terminal 1: Launch simulator
```
ros2 launch chesslab_setup2 chesslab_gz.launch.py launch_rviz:=true
```
- Terminal 2: Action manager server
```
cd /home/mateovp/data/workspaces/colcon_wsFinalWork
source install/setup.bash
ros2 run action_manager_module action_manager_server
```

- Terminal 3: action requirement
```
cd /home/mateovp/data/workspaces/colcon_wsFinalWork
source install/setup.bash
ros2 action send_goal /execute_chess_action action_manager_module_interfaces/action/ExecuteChessAction "{piece_aruco_id: '316', from_cell: 'e2', to_cell: 'a4', is_capture: false, is_castle: false}" --feedback
```

**⚠️ IMPORTANTE:** El action_manager_module fue corregido para usar callbacks asíncronos en lugar de `spin_until_future_complete`, lo cual causaba deadlock. Ver [SOLUCION_ROBOT_INMOVIL.md](../../../SOLUCION_ROBOT_INMOVIL.md) para detalles técnicos.

**🔧 TROUBLESHOOTING:** Si experimentas timeouts en la ejecución de trayectorias, ver [TROUBLESHOOTING_TIMEOUT.md](../../../TROUBLESHOOTING_TIMEOUT.md).

---

## Complete End-to-End Test Sequence

To test the complete system (sensing → planning → execution), run in this order:

**Terminal 1: Gazebo + Sensing Node**
```bash
cd /home/mateovp/data/workspaces/colcon_wsFinalWork
source install/setup.bash
ros2 launch sensing_module chesslab_sensing.launch.py
```

**Terminal 2: IK Server**
```bash
cd /home/mateovp/data/workspaces/colcon_wsFinalWork
source install/setup.bash
ros2 run kinenikros2 kinenik_srv_server
```

**Terminal 3: Planning Node**
```bash
cd /home/mateovp/data/workspaces/colcon_wsFinalWork
source install/setup.bash
ros2 launch planning_module planning_node.launch.py
```

**Terminal 4: Action Manager**
```bash
cd /home/mateovp/data/workspaces/colcon_wsFinalWork
source install/setup.bash
ros2 run action_manager_module action_manager_server
```

**Terminal 5: Execute Test Action**
```bash
cd /home/mateovp/data/workspaces/colcon_wsFinalWork
source install/setup.bash
ros2 action send_goal /execute_chess_action action_manager_module_interfaces/action/ExecuteChessAction "{piece_aruco_id: '316', from_cell: 'E1', to_cell: 'E3'}" --feedback
```

**Useful Verifications:**
- Check detected cell: `ros2 service call /piece_location sensing_module/srv/PieceLocation "{aruco_id: 316}"`
- Monitor trajectory action: `ros2 action list | grep trajectory`
- View robot controllers: `ros2 control list_controllers`
- View active nodes: `ros2 node list`

**Note:** The action_manager_module now uses the FollowJointTrajectory action client (`/joint_trajectory_controller/follow_joint_trajectory`) with **asynchronous callbacks** to control robot movement in Gazebo simulation, ensuring proper execution and feedback **without deadlock**. The previous implementation using `spin_until_future_complete` caused the robot to not move because it blocked the executor. See [SOLUCION_ROBOT_INMOVIL.md](../../SOLUCION_ROBOT_INMOVIL.md) for technical details.
