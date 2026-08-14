from setuptools import find_packages, setup

package_name = 'race_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/green_detection.launch.py']),
        ('share/' + package_name + '/config', ['config/green_detector.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zfy',
    maintainer_email='2495534260@qq.com',
    description='Green endpoint board detector for race track one.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'green_board_detector = race_vision.green_board_detector:main',
        ],
    },
)
