from setuptools import find_packages, setup

package_name = 'ippolit_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dmitry Poklonskiy',
    maintainer_email='dmitry.poklonskiy@kingmakers.com',
    description='Laser-ICP odometry and slam_toolbox map persistence',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'q6a_laser_odom = ippolit_localization.q6a_laser_odom:main',
            'q6a_map_persist = ippolit_localization.q6a_map_persist:main',
        ],
    },
)
