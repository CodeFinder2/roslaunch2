from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['roslaunch2'],
    package_dir={'': 'src'},
    scripts=['script/roslaunch2']
)

setup(**d)
