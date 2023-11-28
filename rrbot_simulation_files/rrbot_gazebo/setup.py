from setuptools import find_packages, setup

package_name = 'Project1'

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
    maintainer='amoore',
    maintainer_email='amoore@wpi.edu',
    description='Assignment 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pd_service = rrbot_simulation.rrbot_gazebo.src.pd_controller_server:main',
            'pd_client = rrbot_simulation.rrbot_gazebo.src.pd_controller_client:main',
        ],
    },
)
