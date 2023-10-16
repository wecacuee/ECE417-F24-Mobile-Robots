import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'jetbot_planning'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=find_packages(exclude=['test']),
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all ini files.
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*ini'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vdhiman',
    maintainer_email='wecacuee@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar = jetbot_planning.astar_node:main',
            'calibrator = jetbot_planning.calibrator:main'
        ],
    },
)
