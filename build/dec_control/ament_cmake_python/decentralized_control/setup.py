from setuptools import find_packages
from setuptools import setup

setup(
    name='decentralized_control',
    version='0.1.0',
    packages=find_packages(
        include=('decentralized_control', 'decentralized_control.*')),
)
