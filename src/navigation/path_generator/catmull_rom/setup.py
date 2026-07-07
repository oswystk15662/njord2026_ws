from setuptools import find_packages, setup

package_name = "catmull_rom_path_smoother"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/catmull_rom_params.yaml"]),
        ("share/" + package_name + "/launch", ["launch/catmull_rom_path_smoother.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="osw",
    maintainer_email="oswystk15662@keio.jp",
    description="Catmull-Rom path smoother for nav_msgs/Path topics",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "catmull_rom_path_smoother_node = "
            "catmull_rom_path_smoother.catmull_rom_path_smoother_node:main",
        ],
    },
)
