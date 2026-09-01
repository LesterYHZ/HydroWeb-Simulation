from setuptools import find_packages, setup
import glob, os

package_name = 'ros2_python_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # List of things you want colcon to make availble
    # "Please put into share/package_name/config things you find with glob(config/*)"
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ozer Ozkahraman',
    maintainer_email='ozero@kth.se',
    description='Am example python package for ros2 with all the bits',
    license='MIT',
    tests_require=['pytest'],
    # This tells ros2 launch commands what to run what its name is
    # Think of it like compilation for python scripts
    # example_node = -> defines the name of the executable
    # package_name.pid_controller_node -> the location of the function
    # :main -> the name of the function to run
    # in the launch file, you would simply use "exec=example_node" in the <node> tag 
    # to launch this specific function in this specific file
    # clearly, this allows you to define _multiple_ functions to run as nodes if you want to
    # while still letting you re-use all the code in your python files.
    entry_points={
        'console_scripts': [
            'example_node = ros2_python_examples.pid_controller_node:main'
        ],
    },
)
