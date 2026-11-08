from glob import glob
from setuptools import setup

setup(
    name='rosbot_exploration',
    version='0.1.0',
    packages=['rosbot_exploration'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/rosbot_exploration']),
        ('share/rosbot_exploration', ['package.xml', 'README.md', 'LICENSE']),
        ('share/rosbot_exploration/launch', glob('launch/*.launch.py')),
        ('share/rosbot_exploration/config', glob('config/*.yaml')),
        ('share/rosbot_exploration/behavior_trees', glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROSbot workspace maintainer',
    maintainer_email='maintainer@example.com',
    description='Standalone frontier exploration for ROS 2 Foxy and Nav2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'explore = rosbot_exploration.exploration_supervisor:main',
    ]},
)
