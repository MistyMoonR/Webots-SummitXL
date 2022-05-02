from setuptools import setup

package_name = 'cartographer'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/cartographer.launch.py','launch/occupancy_grid.launch.py']))
data_files.append(('share/' + package_name + '/config', ['config/carto_2d.lua']))
data_files.append(('share/' + package_name + '/rviz', ['rviz/carto.rviz']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files= data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'remapped = cartographer.remapped:main',
        ],
    },
)
