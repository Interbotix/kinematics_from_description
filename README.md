# Kinematics from Robot Description

## Overview

This package contains a simple Python program that calculates some properties required to run parts of the [Modern Robotics Library](https://github.com/NxRLab/ModernRobotics) from a robot's URDF or robot_description parameter. Specifically, this program calculates the M and Slist parameters required by the MR [IKinSpace](https://github.com/NxRLab/ModernRobotics/blob/4a3d25ae0a64d6c9c65d78345452155e400efa8e/packages/Python/modern_robotics/core.py#L763) function:

- The **M** matrix is the position and orientation of the end effector frame when the robot is in its home position (i.e. all joints are at position 0).
- The **Slist** matrix is the joint screw axes in the end effector frame when the robot is in its home position. This matrix is formatted to have each axis as a column.

## Dependencies

The dependencies of this software package are:

- urdf_parser_py
- numpy
- scipy
- pyyaml

You can use rosdep or the setup file to install the required dependencies.

Note that this package is only compatible with Python3.

## Usage

You must specify three parameters before using this library:

- **namespace** - The namespace of the robot_description parameter and the namespace that each link will be listed under in the URDF. For example, if the robot is named "vx300s", a link may be listed like "vx300s/shoulder_link". If no namespace is used, leave this as an empty string, i.e. `''`.
- **space_frame** - The name given to the link that will serve as the location of the robot's base, or {0} link.
- **body_frame** - The name given to the link that will serve as the location of the end effector.

There are two primary ways to use this library: importing the module, and using rosrun.

### Importing

The KinematicsFromDescription Tool can be imported into your Python3 script.

```python
from kinematics_from_description.kfd import KinematicsFromDescriptionTool as KFD

...

tool = KFD(
    space_frame="space_frame", 
    body_frame="body_frame",
    robot_namespace="namespace")
tool.load_desc_from_file(filepath)
tool.run()
```

### Using rosrun

Since this package is catkin-ized, it can be launched using ROS. 

1. Modify the [config.yaml](./config/config.yaml) file to match your description setup.
2. Launch your robot's robot_description package to load the robot_description parameter into ROS' parameter server.
3. Run the [mr_desc_param.py](./scripts/mr_desc_param.py) script using rosrun.

```console
$ rosrun kinematics_from_description mr_desc_param.py

M:
[[1.       0.       0.       0.248575]
 [0.       1.       0.       0.      ]
 [0.       0.       1.       0.1931  ]
 [0.       0.       0.       1.      ]]

Slist:
[[ 0.      0.      1.      0.      0.      0.    ]
 [ 0.      1.      0.     -0.0931  0.      0.    ]
 [ 0.      1.      0.     -0.1931  0.      0.035 ]
 [ 0.      1.      0.     -0.1931  0.      0.135 ]]
```

## Contributors

- [Luke Schmitt](https://github.com/LSinterbotix)

## Future Work

- Include other useful variables like Blist
- Allow for joint types other than revolute
- Incorporate error handling
- Improve robustness
