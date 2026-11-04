"""Install the pysoarm ROS 2 node as an ament_python package."""

from setuptools import setup

package_name = "pysoarm_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "pysoarm>=0.1"],
    zip_safe=True,
    maintainer="pysoarm maintainers",
    maintainer_email="maintainers@example.com",
    description="ROS 2 node for SO-100/SO-101 arms driven by pysoarm",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_node = pysoarm_ros2.node:main",
        ],
    },
)
