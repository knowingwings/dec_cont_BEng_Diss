from setuptools import setup, find_packages
import os
from glob import glob
from setuptools.dist import Distribution

package_name = 'decentralized_control'

# Override get_version to avoid version canonicalization
Distribution.get_version = lambda self: '1.0'

setup(
    name='dec_control',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/dec_control']),
        ('share/dec_control', ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tom Le Huray',
    maintainer_email='thomas.lehuray@outlook.com',
    description='Decentralized control for dual mobile manipulators',
    license='TODO: License declaration'
)