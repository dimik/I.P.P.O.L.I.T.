from setuptools import find_packages, setup

package_name = 'ippolit_safety'

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
    description='Cliff/stair safety: wheel-drop e-stop, MiDaS virtual-obstacle cliff scan',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
