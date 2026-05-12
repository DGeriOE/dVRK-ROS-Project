import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_course'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='geri',
    maintainer_email='geri@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hello = ros2_course.hello:main',
            'turtlesim_controller = ros2_course.turtlesim_controller:main',
            'psm_grasp = ros2_course.psm_grasp:main',
            'psm_grasp2 = ros2_course.psm_grasp2:main',
            'dummy_marker = ros2_course.dummy_marker:main',
            'interactive_marker = ros2_course.interactive_marker:main',
            'psm_interactive_grasp = ros2_course.psm_interactive_grasp:main',
            
        ],
    },
)
