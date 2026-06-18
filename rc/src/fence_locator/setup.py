from setuptools import find_packages, setup

package_name = 'fence_locator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'launch', 'resource']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [
            'launch/launch_fence.py',
            'launch/launch_fence.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    entry_points={
        'console_scripts': [
            'fence_locator = fence_locator.fence_locator_node:main',
        ],
    },
)
