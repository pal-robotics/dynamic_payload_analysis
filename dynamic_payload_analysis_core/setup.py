from setuptools import find_packages, setup

package_name = 'dynamic_payload_analysis_core'

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
    maintainer='Enrico Moro',
    maintainer_email='enrimoro003@gmail.com',
    description='This package implements core functionalities for dynamic payload analysis in robotics, focusing on torque calculations and external force handling.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
