from setuptools import setup

package_name = 'gnc_offboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GNC team',
    maintainer_email='gnc@example.com',
    description='PX4 offboard control and field-frame utilities for AVC Mission Bird Dog',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_demo = gnc_offboard.mission_demo:main',
            'mission_lawnmower = gnc_offboard.mission_lawnmower:main',
            'mission_spiral_in = gnc_offboard.mission_spiral_in:main',
            'mission_spiral_out = gnc_offboard.mission_spiral_out:main',
            'mission_frontier = gnc_offboard.mission_frontier:main',
            'mission_zones = gnc_offboard.mission_zones:main',
            'mission_hybrid = gnc_offboard.mission_hybrid:main',
            'camera_fov = gnc_offboard.camera_fov:main',
        ],
    },
)
