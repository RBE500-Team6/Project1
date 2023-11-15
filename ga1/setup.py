from setuptools import find_packages, setup

package_name = 'ga1'

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
    maintainer='jesseb08',
    maintainer_email='jeburns@wpi.edu',
    description='Assignment 1',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'fk_node = ga1.fk_node:main',
        ],
    },
)
