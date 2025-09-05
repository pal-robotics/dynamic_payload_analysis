from setuptools import find_packages, setup
import glob

package_name = 'dynamic_payload_analysis_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/examples' + '/urdf' + '/h1_description' + '/urdf', ['examples/urdf/h1_description/urdf/h1.urdf']),
        ('share/' + package_name + '/examples' + '/urdf' + '/h1_description' + '/urdf', ['examples/urdf/h1_description/urdf/h1_with_hand.urdf']),
        ('share/' + package_name + '/examples' + '/urdf' + '/h1_description' + '/meshes', glob.glob('examples/urdf/h1_description/meshes/*')),
    ],
    zip_safe=True,
    maintainer='Enrico Moro',
    maintainer_email='enrimoro003@gmail.com',
    description='This package implements functionalities for dynamic payload analysis,'
                'focusing on torque calculations and external force handling.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    install_requires=[
        'setuptools',
    ],
    entry_points={
        'console_scripts': [
        ],
    },
)
