from setuptools import setup
from glob import glob
import os

package_name = 'dec_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
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
    license='TODO: License declaration',    tests_require=['pytest'],
    python_requires='>=3.8',
    entry_points={
        'console_scripts': [
            'task_manager_node = decentralized_control.task_manager.task_manager_node:main',
            'auction_node = decentralized_control.auction.auction_node:main',
            'consensus_node = decentralized_control.consensus.consensus_node:main',
            'recovery_node = decentralized_control.recovery.recovery_node:main',
            'execution_controller = decentralized_control.execution.execution_controller:main',
            'communication_middleware = decentralized_control.communication_middleware:main',
            'metrics_collector = decentralized_control.metrics_collector:main',
            'comparative_analysis = decentralized_control.comparative_analysis:main',
            'experiment_runner = decentralized_control.experiment_runner:main',
            'industrial_scenario_generator = decentralized_control.industrial_scenario_generator:main',
            'task_dependency_analyzer = decentralized_control.task_dependency_analyzer:main',
            'collaborative_task_executor = decentralized_control.collaborative_task_executor:main',
            'math_proof_validator = decentralized_control.math_proof_validator:main',
            # Gazebo integration
            'robot_model_spawner = decentralized_control.robot_model_spawner:main',
            'gazebo_physics_configurator = decentralized_control.gazebo_physics_configurator:main',
            'assembly_task_visualizer = decentralized_control.assembly_task_visualizer:main',
            # Tests
            'test_communication = decentralized_control.test.test_communication:main',
            'test_auction = decentralized_control.test.test_auction:main',
            'test_consensus = decentralized_control.test.test_consensus:main',
            'test_recovery = decentralized_control.test.test_recovery:main',
            'test_task_manager = decentralized_control.test.test_task_manager:main',
            'test_execution = decentralized_control.test.test_execution:main',
            'test_metrics = decentralized_control.test.test_metrics:main',
            'test_comparative = decentralized_control.test.test_comparative:main',
        ],
    },
)