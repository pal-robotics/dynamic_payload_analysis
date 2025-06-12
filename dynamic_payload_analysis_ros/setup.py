from setuptools import find_packages, setup

package_name = 'dynamic_payload_analysis_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='morolinux',
    maintainer_email='enrimoro003@gmail.com',
    description='This package provides tools for dynamic payload analysis in robotics with a focus on torque calculations and external force handling.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_rviz_visualization = dynamic_payload_analysis_ros.rviz_visualization:main',
            'node_rviz_visualization_menu = dynamic_payload_analysis_ros.rviz_visualization_menu:main',
        ],
    },
)
