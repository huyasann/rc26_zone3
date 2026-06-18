from setuptools import find_packages, setup

package_name = 'zone_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='inkc',
    maintainer_email='inkc008@gmail.com',
    description=(
        'Zone2 (Merlin facade) + Zone3 (3x3 grid) + KFS Grid color detector nodes.'
    ),
    license='MIT',
    extras_require={},
    entry_points={
        'console_scripts': [
            'zone2_detector = zone_detection.zone2.detector_node:main',
            'zone3_localizer = zone_detection.zone3.localizer_node:main',
            'zone3_platform_fitter = zone_detection.zone3_fit.localizer_node:main',
            'kfs_grid_detector = zone_detection.kfs_grid.detector_node:main',
            'kfs_grid_detector_qt = zone_detection.kfs_grid.detector_node:main_qt',
        ],
    },
)
