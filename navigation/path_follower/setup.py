from setuptools import find_packages, setup

package_name = "path_follower"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/path_follower.launch.py"]),
        ("share/" + package_name + "/config", ["config/path_follower_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="osw",
    maintainer_email="oswystk15662@keio.jp",
    description="Pure pursuit path follower for Njord simulation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "path_follower_node = path_follower.pure_pursuit_follower_node:main",
        ],
    },
)