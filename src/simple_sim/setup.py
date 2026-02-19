from setuptools import find_packages, setup

package_name = 'simple_sim'

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
    maintainer='osw',
    maintainer_email='oswystk15662@keio.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_sim_node = simple_sim.simple_sim_node:main',
            'simple_purepursuit = simple_sim.simple_purepursuit:main',
            'square_path = simple_sim.square_path:main',
            'send_square_goal = simple_sim.send_square_goal:main',
            'send_square_goal_v2 = simple_sim.send_square_goal_v2:main',
        ],
    },
)
