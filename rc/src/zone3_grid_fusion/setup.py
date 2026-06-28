from setuptools import find_packages, setup

package_name = "zone3_grid_fusion"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="ros@todo.todo",
    description="Zone3 corner TF and nine-grid cloud fusion helper.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "zone3_grid_fusion = zone3_grid_fusion.zone3_grid_fusion_node:main",
            "depth_grid_plane_probe = zone3_grid_fusion.depth_grid_plane_probe:main",
        ],
    },
)
