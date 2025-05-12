from setuptools import find_packages
import os
from glob import glob

package_name = 'decentralized_control'  # This should match your Python package directory name

setup_args = {
    'name': 'dec_control',  # ROS package name
    'version': '1.0',       # Simple version number
    'packages': [package_name],
    'package_dir': {'': '.'},
    'include_package_data': True,
    'zip_safe': True,
    'author': 'Tom Le Huray',
    'author_email': 'thomas.lehuray@outlook.com',
    'maintainer': 'Tom Le Huray',
    'maintainer_email': 'thomas.lehuray@outlook.com',
    'description': 'Decentralized control for dual mobile manipulators',
    'license': 'TODO: License declaration',
    'install_requires': ['setuptools'],
    'data_files': [
        ('share/ament_index/resource_index/packages', ['resource/dec_control']),
        ('share/dec_control', ['package.xml']),
        (os.path.join('share', 'dec_control', 'launch'), glob('launch/*')),
        (os.path.join('share', 'dec_control', 'config'), glob('config/*')),
        (os.path.join('share', 'dec_control', 'models'), glob('models/**/*', recursive=True)),
        (os.path.join('share', 'dec_control', 'worlds'), glob('worlds/*')),
    ],
}

from setuptools import setup
setup(**setup_args)