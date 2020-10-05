# Dummy ReconCell ROS action servers
This package was created in order to facilitate development of tools around the ReconCell ROS infrastructure. Uppon launching the launchfiles it brings up the Action Servers ~~and Services~~ (work in progress). Developers can then write various programs that interact with the these components and make sure the communication between their programs and the cell will work.
# To do
- Enable Github CI
- Add services

# Table of contents

- [Dummy ReconCell ROS action servers](#dummy-reconcell-ros-action-servers)
- [To do](#to-do)
- [Table of contents](#table-of-contents)
- [Quick How-to](#quick-how-to)
- [Detailed description](#detailed-description)
- [Examples](#examples)
  - [Simple test](#simple-test)
  - [Simle test with namespaces](#simle-test-with-namespaces)
  - [Launchfile bringup](#launchfile-bringup)
  - [Contributing](#contributing)
          - [<sup>1</sup> catkin tools](#sup1sup-catkin-tools)
          - [<sup>2</sup> wstool](#sup2sup-wstool)
          - [<sup>3</sup> rosdep](#sup3sup-rosdep)
          - [rosdep != wstool](#rosdep--wstool)

# Quick How-to

First of all, navigate to the `src` directory in your catkin workspace. Then execute the following commands:
```
git pull git@github.com:ReconCycle/dummy_interfaces.git
wstool init src/
wstool merge -t src src/dummy_interfaces/rosdep.rosinstall
wstool update -t src
rosdep install --from-paths src/dummy_interfaces --ignore-src -r -y
catkin build
source devel/setup.bash
```

and finally:

```
roslaunch dummy_interfaces launch.launch
```

**Note:** If you are missing any of the tools used above (
catkin tools<sup>[1](#1-catkin-tools)</sup>,
wstool<sup>[2](#2-wstool)</sup>,
rosdep<sup>[3](#3-rosdep)</sup>
), see the links at the bottom of this page.

# Detailed description

As mentioned above, this package contains "dummy" executables that, when launched, on the outside behave the similarly as the real Action Servers and ~~Services~~. However, be aware, that the dummy Action Servers that would trigger motions on the real robot do not perform any safety checks that would normally be performed on real cell. This is something that has to be taken in account when writing code around the dummy Action Servers.

The currently implemented action servers are:
* Joint space motion with a trapezoidal speed profile
* Joint space motion with a trapezoidal speed profile but with a Cartesian space target
* Cartesian space straight line motion with a minimum jerk profile

Launching all the action server is done by launching a file the appropriate launch file: `launch.launch`. This will launch Action Servers and ~~Services~~ for two robots and put them in their appropriate namespace.

# Examples

## Simple test
Bring up one of the Action Servers, for example the `joint_trap_vel_action_server`:

```
rosrun dummy_interfaces dummy_joint_trap_vel_action_server.py
```

and test it by running the script:

```
rosrun dummy_interfaces test_dummy_action_servers.py
```

What you should see is the test script call the Action Server twice, once normally and once with a preemtpion. It will then exit with the error:
```
[INFO] [1548078363.246895]: Waiting for cart_trap_vel_action ...
[ERROR] [1548078365.250193]: Exception during the test phase:
Action server unreachable !!
```

This is OK because the test script usually tests all three Action Servers and since we started only one it cannot communicate with the other two.


## Simle test with namespaces


If you want to put them in a namespace just append `__ns:=<NAMESPACE>` to the end of the rosrun command:

```
rosrun dummy_interfaces dummy_joint_trap_vel_action_server.py __ns:='panda_1'
```

and test it by running the script:

```
rosrun dummy_interfaces test_dummy_action_servers.py __ns:='panda_1'
```

The result should be the same as with the simple test.


## Launchfile bringup

In the `/launch` directory there are three launchfiles. `main_robot.launch` and `second_robot.launch` start the Action Servers and ~~Services~~ for the `panda_1` and `panda_2` robot respectively. The `launch.launch` file launches both robots in their namespaces. So in order to create an environment that represents the one on the real ReconCell you should launch the main launchfile i.e. `launch.launch`:

```
roslaunch dummy_interfaces launch.launch
```

Then you can run the test script by adding the namespace parameter and it should perorm the calls to all three Action Servers and ~~Services~~:

```
rosrun dummy_interfaces test_dummy_action_servers.py __ns:='panda_1'
```

## Contributing
Feel free to contribute to the project with pull requests and point out bugs or suggestions via the Issue tracker.


<br/>

---

###### <sup>1</sup> catkin tools
>This Python package provides command line tools for working with the catkin meta-buildsystem and catkin workspaces. These tools are separate from the Catkin CMake macros used in Catkin source packages.

Website: https://catkin-tools.readthedocs.io/en/latest/

###### <sup>2</sup> wstool
>Command-line tools for maintaining a workspace of projects from multiple version-control systems.

Website: http://wiki.ros.org/wstool

###### <sup>3</sup> rosdep

> rosdep is a command-line tool for installing system dependencies.

Website: http://wiki.ros.org/rosdep

###### rosdep != wstool

The difference between `rosdep` and `wstool` is that `rosdep` can install system dependencies by reading the `package.xml` file from the official [ROS repositories](https://github.com/ros/rosdistro), while `wstool` installs packages from a `.rosinstall` file and can fetch them from various user-defined version-control systems.
