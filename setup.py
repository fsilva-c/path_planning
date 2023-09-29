from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# catkin build

setup_args = generate_distutils_setup(
    version="0.0.0",
    packages=['classic_path_planner', 'geometry', 'RL_path_planner', 'uav_interface'],
    package_dir={'': 'src'},
    package_data={'fs_path_planning': ['msg/*.msg']})

setup(**setup_args)