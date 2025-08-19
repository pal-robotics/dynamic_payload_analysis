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
        ('share/' + package_name + '/launch', ['launch/dyn_payload_analysis.launch.py']),
        ('share/' + package_name + '/launch', ['launch/examples/dyn_analysis_payload_tiago_pro.launch.py']),
        ('share/' + package_name + '/launch', ['launch/examples/dyn_analysis_payload_talos.launch.py']),
        ('share/' + package_name + '/launch', ['launch/examples/dyn_analysis_payload_franka.launch.py']),
        ('share/' + package_name + '/launch', ['launch/examples/dyn_analysis_payload_ur_robots.launch.py']),
        ('share/' + package_name + '/launch', ['launch/examples/dyn_analysis_payload_unitree_h1.launch.py']),
        ('share/' + package_name + '/config' + '/rviz', ['config/rviz/tiago_pro_dyn_analysis.rviz']),
        ('share/' + package_name + '/config' + '/rviz', ['config/rviz/talos_dyn_analysis.rviz']),
        ('share/' + package_name + '/config' + '/rviz', ['config/rviz/franka_dyn_analysis.rviz']),
        ('share/' + package_name + '/config' + '/rviz', ['config/rviz/ur_robots_dyn_analysis.rviz']),
        ('share/' + package_name + '/config' + '/rviz', ['config/rviz/h1_dyn_analysis.rviz']),

    ],
    zip_safe=True,
    maintainer='Enrico Moro',
    maintainer_email='enrimoro003@gmail.com',
    description='This package provides graphics tools in Rviz for dynamic payload analysis in robotics with a focus on torque calculations and external force handling.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    install_requires=[
        'setuptools',
        'numpy'
    ],
    entry_points={
        'console_scripts': [
            'node_rviz_visualization_menu = dynamic_payload_analysis_ros.rviz_visualization_menu:main',
        ],
    },
)
