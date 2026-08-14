from setuptools import find_packages, setup

package_name = 'race_control'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/finish_autonomy.yaml',
            'config/map_search_autonomy.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zfy',
    maintainer_email='2495534260@qq.com',
    description='One-button autonomous control for race track one.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'finish_autonomy = race_control.finish_autonomy:main',
            'map_search_autonomy = race_control.map_search_autonomy:main',
            'twist_priority_mux = race_control.twist_priority_mux:main',
            'moving_obstacle = race_control.moving_obstacle:main',
            'race_metrics = race_control.race_metrics:main',
            'race_start_key = race_control.race_start_key:main',
            'twist_to_twist_stamped = race_control.twist_to_twist_stamped:main',
        ],
    },
)
