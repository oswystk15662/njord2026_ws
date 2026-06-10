from setuptools import find_packages, setup

package_name = "path_generator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/path_generator.launch.py"]),
        ("share/" + package_name + "/config", ["config/path_generator_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="osw",
    maintainer_email="oswystk15662@keio.jp",
    description="Catmull-Rom path generator for Njord simulation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "path_generator_node = path_generator.path_generator_node:main",
        ],
    },
)