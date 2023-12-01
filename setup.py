from setuptools import find_packages, setup

package_name = 'turtle_pose_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='august',
    maintainer_email='augustjf@stud.ntnu.no',
    description='Initial pose estimation in map',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_pose_estimation = turtle_pose_estimation.example_navigation:main'
        ],
    },
)
