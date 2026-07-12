from setuptools import find_packages, setup

package_name = 'ippolit_perception'

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
    description='Vision (YOLO+ByteTrack+MiDaS) and semantic object map',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'q6a_announce = ippolit_perception.q6a_announce:main',
            'q6a_vision = ippolit_perception.q6a_vision:main',
            'q6a_objmap = ippolit_perception.q6a_objmap:main',
        ],
    },
)
