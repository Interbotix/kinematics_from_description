#!/usr/bin/env python3

import yaml
import os
import numpy as np
from kinematics_from_description.kfd import KinematicsFromDescriptionTool as KFD

if __name__ == "__main__":
    filepath = os.path.dirname(os.path.abspath(__file__))
    with open("%s/../config/config.yaml" % filepath) as config_file:
        try:
            config = yaml.safe_load(config_file)
        except yaml.YAMLError as e:
            print(e)
            exit()

    np.set_printoptions(precision=config["precision"], suppress=True)

    tool = KFD(
        space_frame=config["space_frame"],
        body_frame=config["body_frame"],
        robot_namespace=config["namespace"],
    )

    tool.load_desc_from_param()
    tool.run()

    print("\nM:")
    print(tool.get_M())

    print("\nSlist:")
    print(tool.get_Slist())
