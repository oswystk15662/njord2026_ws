#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Setup file for pcl_det package."""

from setuptools import find_packages, setup

setup(
    name='pcl_det',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pcl_det']),
        ('share/pcl_det', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='osw',
    author_email='oswystk15662@keio.jp',
    description='PCL-based detection for underwater buoys',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'pcl_bouy_det=pcl_det.pcl_bouy_det:main',
        ],
    },
)
