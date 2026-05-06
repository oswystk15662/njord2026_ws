import os
import glob
from setuptools import setup

package_name = 'micon_agent_esp32'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob.glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
         glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='osw',
    author_email='oswystk15662@keio.jp',
    maintainer='osw',
    maintainer_email='oswystk15662@keio.jp',
    keywords=['ROS2', 'micro-ROS', 'ESP32'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='micro ROS agent for ESP32 communication',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'micon_esp32_agent = micon_agent_esp32.agent:main',
        ],
    },
)
