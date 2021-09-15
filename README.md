# walknet-curvewalking
Reimplementation of the Walknet decentralized architecture for six-legged robots with the goal of adjusting it for curve-walking.

## References
The simulation with which this code was developed can be found here:
<https://github.com/kkonen/phantomx_gazebo>.

It requires the robot description which can be found here:
<https://github.com/kkonen/phantomx_description>.

And to control the robot the ROS controller from HumanRobotics:
<https://github.com/HumaRobotics/phantomx_control>.

The walknet implementation which this code is based on and where parts of the code are taken from can be found here:
<https://github.com/malteschilling/cognitiveWalker>.

## Running the scripts
This Project is a catkin-project. It needs to be cloned into the src directory of a catkin workspace and installed by running `catkin_make` in the root directory of that workspace.
It requires that the phantomX simulation is running and publishes the current joint values to the `/phantomx/j_<joint-name_leg-name>_position_controller/state`-topics.
The walknet-controller will send joint angles as commands to the `/phantomx/j_<joint-name_leg-name>_position_controller/command`-topics.
The controller logs the legs endeffector positions during stance phases, the durations of the stance phases and some additional information including the walking duration and percentage of unstable controller steps.
Per default this requres a logs directory in the catkin workspace next to the src directory with the following structure:

```
catkin_ws
│
└───src
|   ...
│   
└───logs
    │   durations
    |   stability
```

The directory the log files are saved in can be modified using the ros-parameter ~name (str, default: logs/).
However, the provided directory always has to contain the subdirectories dirations and stability.
If the robots position in the simulation is supposed to be tracked using the DataCollector.py a subdirectory called position is additionally needed.
As for the other log files a custom logging directory for the position subdirectory other than the logs directory can be set using the ros-parameter ~name (str, default: logs/).

If these requirements are fulfilled the walknet-controller can be started by running:
```python
rosrun walknet_curvewalking robot_controller.py
```

The robot can then be controlled by publishing robot_controll messages:
```python
rosrun walknet_curvewalking robot_control_pub.py _speed:=0.01 _direction:=0.0
```
(This commands the robot to walk straight forward \_direction:=0.0 at a velocity of 0.01m/s \_speed:=0.01)
To stop the run another controll message has to be send:
```python
rosrun walknet_curvewalking robot_control_pub.py _speed:=0.0 _direction:=0.0
```

The walknet-controller (robot_controller.py) takes a number of rosparameter which are listed below:
~aepShift (boolean, default: True)
  True if the AEPs should be shifted in y direction (father to the side and closer to the robot body).
~aepShiftX (boolean, default: True)
  True if the AEPs should be shifted in x direction (forward and backward along the robot body).
~back (boolean, default: True)
  True if an additonal pull vector should be applied to the back of the robot body model to aid in turning the robot.
~duration (float, default: 0)
  How long the robot should walk. If 0 the robot walks until it is commanded to stop by a robot_control message.
~innerStep (boolean, default: True)
  If the step length of the inside legs should be decreased for certain curves.
~stepLength (boolean, default: True)
  True if the step length should determine when the leg switches from stance to swing.
  If False a threshold in x direction determines when the leg switches from stance to swing.
~walk (boolean, default: True)
  Whether the robot should wait for a control command.
  If False the controller only moves the legs into the start position and ends without walking.
~swing (boolean, default: True)
  Whether the robot shoul swing it's legs.
  If False the robot only move the body with the body model and stops once one of the legs should be lifted of the ground.
  In this case the initial position of the robot is changed to a position with all legs relatively far forwards.
~name (str, default: "logs/")
  Path to the directory the log files are should be saved in. This directory needs to contain the subdirectories durations and stability.


The DataCollector can also be customized using rosparameter:
~circles (int, optional)
  The number of circles the robot should walk.
~distance (float, optional)
  The distance the robot should walk in meter.
~name (str, default: "logs/")
  Path to the directory the log files are should be saved in. This directory needs to contain the subdirectory position.

If neither distance nor circles is provided the robot will either walk for the duration set as rosparameter for the robot_controller or until a robot_control message is received commanding the robot to stop.

## Batch recording

To simulate multiple runs the record_data.py can be used.
Since it starts the walknet-controller it requires the simulation to be running.
In the main of the record_data.py the directorys for the loging files and the Walknet versions have to be defined.
Additionally, the walking directions and velocities have to be provided.
For example:

```python
root_dir = "logs/"
trials = [
        {"name": root_dir + "original/", "length": False, "aep_y": False, "aep_x": False, "decrease": False, "pull_at_back": False}
        {"name": root_dir + "curvewalking/", "length": True, "aep_y": True, "aep_x": True, "decrease": True, "pull_at_back": True}
    ]
direction = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.57]
speed = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06]
```
This would simulate the initial Walknet implementation and the Version with all curvewalking modifications active each for every combination of provided directions in rad and provided velocities (speed variable) in m/s and save the log files in the durations, stability and position subdirectories of the logs directory.
Using the record_data.py the subdirectories are automatically created.

## Branches

The widowX and widowX-stability-enforcement branch are for use with the [WidowX-simulation](https://github.com/Interbotix/interbotix_ros_crawlers/tree/main/interbotix_ros_xshexapods/interbotix_xshexapod_gazebo) or the WidowX robot.
The widowX branch contains the Walknet version from the main branch adapted to the WidowX-robot.
The widowX-stability-enforcement branch contains an additional stability enforcement which increases the static stability of the system.
The phantomx branch is for use with the PhantomX robot, but has been discontinued and was never succesfully tested on the Robot.
The other branches are backups or tests of features that are incorporated in some of the major branches or discontinued.
