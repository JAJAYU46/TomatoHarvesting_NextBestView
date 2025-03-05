from setuptools import find_packages
from setuptools import setup

setup(
    name='message_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('message_interfaces', 'message_interfaces.*')),
)
