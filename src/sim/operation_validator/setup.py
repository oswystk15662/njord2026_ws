from setuptools import find_packages, setup

package_name = "operation_validator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/operation_validator.launch.py"]),
        ("share/" + package_name + "/config", ["config/validator_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="osw",
    maintainer_email="oswystk15662@keio.jp",
    description="Operation validator for simulation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "operation_validator_node = operation_validator.validator_node:main",
        ],
    },
)
