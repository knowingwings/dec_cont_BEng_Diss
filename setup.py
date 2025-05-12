from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'dec_control'

setup(
    name=package_name,
    version='0.0.1',  # Simple version number
    packages=find_packages(),  # Automatically find packages
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tom Le Huray',
    maintainer_email='thomas.lehuray@outlook.com',
    description='Decentralized control for dual mobile manipulators',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager_node = decentralized_control.task_manager.task_manager_node:main',
            'auction_node = decentralized_control.auction.auction_node:main',
            'consensus_node = decentralized_control.consensus.consensus_node:main',
            'recovery_node = decentralized_control.recovery.recovery_node:main',
            'execution_controller = decentralized_control.execution.execution_controller:main',
            'communication_middleware = decentralized_control.communication_middleware:main',
        ],
    },
)