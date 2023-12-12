from setuptools import find_packages, setup

package_name = 'rrbot_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amoore',
    maintainer_email='amoore@wpi.edu',
    description='Assignment 2 PD Controller',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'pd_service = rrbot_python.pd_controller_server:main',
            'pd_client = rrbot_python.pd_controller_client:main',
            'velocities_service = rrbot_python.velocities_server:main',
        ],
    },
)
