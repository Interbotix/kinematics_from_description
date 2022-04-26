from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["kinematics_from_description"],
    package_dir={"": "src"},
    scripts=["scripts/mr_desc_param"],
    classifiers=[
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: BSD License",
        "Natural Language :: English",
        "Programming Language :: Python :: 3",
        "Topic :: Education",
        "Topic :: Scientific/Engineering",
    ],
    install_requires=["numpy", "scipy>=1.2.0", "urdf_parser_py", "pyyaml"],
)

setup(**d)
