from glob import glob

from setuptools import setup

package_name = 'rosbot_telegram_ai'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fran Fuentes',
    maintainer_email='franfuentes2004@gmail.com',
    description='Bot de Telegram con Claude para el ROSbot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'telegram_ai_node = rosbot_telegram_ai.telegram_ai_node:main',
        ],
    },
)
