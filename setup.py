from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'enme480_lab_fk'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='enme480_docker',
    maintainer_email='enme480_docker@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur3e_fk = enme480_lab_fk.ur3e_fk:main',
            'ur3e_ik = enme480_lab_fk.ur3e_ik:main'
        ],
    },
)
