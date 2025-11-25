from setuptools import find_packages, setup

package_name = 'ri_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/camera_subscriber.launch.py',
            'launch/autonomous_navigation.launch.py'
        ]),
    ],
    install_requires=['setuptools', 'opencv-python'],
    zip_safe=True,
    maintainer='akshat',
    maintainer_email='akshat22050@iiitd.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_subscriber = ri_pkg.camera_subscriber:main',
            'yolo_clip_camera_node = ri_pkg.yolo_clip_camera_node:main',
            'yolo_improved_vlm_node = ri_pkg.yolo_improved_vlm_node:main',
            'autonomous_navigation_node = ri_pkg.autonomous_navigation_node:main',
            'navigation_controller = ri_pkg.navigation_controller:main',
            'navigation_diagnostics = ri_pkg.navigation_diagnostics:main',
            'test_rotation = ri_pkg.test_rotation:main',
            'coordinate_mapping = ri_pkg.coordinate_mapping:main',
        ],
    },
)
