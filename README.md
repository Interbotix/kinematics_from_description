# Kinematics from Robot Description ![CI Status](https://github.com/Interbotix/kinematics_from_description/actions/workflows/ci.yaml/badge.svg)

## Overview

This package contains a simple Python program that calculates some properties required to run parts of the [Modern Robotics Library](https://github.com/NxRLab/ModernRobotics) from a robot's URDF or robot_description parameter. Specifically, this program calculates the M and Slist parameters required by MR kinematics functions:

- The **M** matrix is the end effector configuration in SE(3) when the robot is in its home position (i.e. all joints are at position 0).
- The **Slist** matrix is the joint screw axes expressed in the space frame when the robot is in its home position. This matrix is formatted to have each axis as a column.

Note that this package is only compatible with ROS Noetic and Python3. However, it can be used other ROS distributions by installing using the requirements.txt and using the Importing method described below.

## Dependencies

The dependencies of this software package are:

- urdf_parser_py
- numpy
- scipy>=1.2.0 (for the scipy.spatial.transform module)
- pyyaml

You can use rosdep (only on Noetic), the requirements.txt file, or the setup.py script to install the required dependencies.

## Usage

You must specify three parameters before using this library:

- **namespace** - The namespace of the robot_description parameter and the namespace that each link will be listed under in the URDF. For example, if the robot is named `vx300s`, a link may be named something like `vx300s/shoulder_link`. If no namespace is used, leave this as an empty string, i.e. `''`.
- **space_frame** - The name given to the link that will serve as the location of the robot's base, or {0} link.
- **body_frame** - The name given to the link that will serve as the location of the end effector.

There are two primary ways to use this library: importing the module, and using rosrun.

### Importing

The KinematicsFromDescription Tool can be imported into your Python script. The Tool accepts a single positional argument - a dictionary with the keys `space_frame`, `body_frame`, and `namespace`. See the [config file](config/config.yaml) for details.

```python
from kinematics_from_description.kfd import KinematicsFromDescriptionTool as KFD

...

tool = KFD(
    {
        space_frame: "space_frame",
        body_frame: "body_frame",
        namespace: "namespace"
    }
tool.load_desc_from_file(filepath)
tool.run()
```

### Using rosrun

Since this package is catkin-ized, it can be launched using ROS.

#### Using a configuration file:
 - Modify the [config.yaml](./config/config.yaml) file to match your description setup.
 - Launch your robot's robot_description package to load the robot_description parameter into ROS' parameter server.
 - Run the [mr_desc_param](./scripts/mr_desc_param) script using rosrun, specifying the location of the config file.

```console
$ rosrun kinematics_from_description mr_desc_param -f /path/to/config.yaml

Loading configs from '/path/to/config.yaml'
Using configs:
        space_frame: base_link
        body_frame:  ee_gripper_link
        namespace:   wx200
        precision:   6

The MR Descriptions are as follows:

M:
array([[1.      , 0.      , 0.      , 0.408575],
       [0.      , 1.      , 0.      , 0.      ],
       [0.      , 0.      , 1.      , 0.31065 ],
       [0.      , 0.      , 0.      , 1.      ]])

Slist:
array([[ 0.     ,  0.     ,  1.     ,  0.     ,  0.     ,  0.     ],
       [ 0.     ,  1.     ,  0.     , -0.11065,  0.     ,  0.     ],
       [ 0.     ,  1.     ,  0.     , -0.31065,  0.     ,  0.05   ],
       [ 0.     ,  1.     ,  0.     , -0.31065,  0.     ,  0.25   ],
       [ 1.     ,  0.     ,  0.     ,  0.     ,  0.31065,  0.     ]]).T

Paste these arrays into your mr_descriptions.py file to use in the Interbotix Python-ROS API.
```

#### Using command line arguments.
 - Specify the `namespace`, `body_frame`, and `space_frame` configs from the command line.

```console
$ rosrun kinematics_from_description mr_desc_param -n wx200 -b ee_gripper_link -s base_link -p 6

Using configs:
        space_frame: base_link
        body_frame:  ee_gripper_link
        namespace:   wx200
        precision:   6

The MR Descriptions are as follows:

M:
array([[1.      , 0.      , 0.      , 0.408575],
       [0.      , 1.      , 0.      , 0.      ],
       [0.      , 0.      , 1.      , 0.31065 ],
       [0.      , 0.      , 0.      , 1.      ]])

Slist:
array([[ 0.     ,  0.     ,  1.     ,  0.     ,  0.     ,  0.     ],
       [ 0.     ,  1.     ,  0.     , -0.11065,  0.     ,  0.     ],
       [ 0.     ,  1.     ,  0.     , -0.31065,  0.     ,  0.05   ],
       [ 0.     ,  1.     ,  0.     , -0.31065,  0.     ,  0.25   ],
       [ 1.     ,  0.     ,  0.     ,  0.     ,  0.31065,  0.     ]]).T

Paste these arrays into your mr_descriptions.py file to use in the Interbotix Python-ROS API.
```

See usage details with the `-h` flag.
```console
$ rosrun kinematics_from_description mr_desc_param -h

usage: mr_desc_param [-h] [--file FILE] [--space_frame SPACE_FRAME]
                     [--body_frame BODY_FRAME] [--namespace NAMESPACE]
                     [--precision PRECISION]

A program that will parse a robot's kinematic properties from its
`/robot_description` rosparam.

optional arguments:
  -h, --help            show this help message and exit
  --file FILE, -f FILE  The path to the config file to use.
  --space_frame SPACE_FRAME, -s SPACE_FRAME
                        The name given to the link that will serve as the
                        location of the robot's base, or {0} link. This is
                        typically `base_link`.
  --body_frame BODY_FRAME, -b BODY_FRAME
                        The name given to the link that will serve as the
                        location of the end effector. This is typically
                        `ee_gripper_link`.
  --namespace NAMESPACE, -n NAMESPACE
                        The namespace under which the `/robot_description`
                        rosparam will be found, as well as the names of each
                        link (i.e. `/namespace/robot_description`, and
                        `namespace/base_link`. (default: )
  --precision PRECISION, -p PRECISION
                        The precision with which to print the kinematic
                        properties. (default: 6)
```

## Contributors

- [Luke Schmitt](https://github.com/LSinterbotix)

## Future Work

- Include other useful variables like Blist
- Allow for joint types other than revolute
- Allow for different rosparam and joint namespaces
