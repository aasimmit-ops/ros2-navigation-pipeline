from setuptools import find_packages, setup

package_name = 'path_navigation'

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
    maintainer='aasim',
    maintainer_email='aasim.mit@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
  entry_points={
    'console_scripts': [
        'smoothing_node = path_navigation.smoothing_node:main',
        'trajectory_node = path_navigation.trajectory_node:main',
        'controller_node = path_navigation.controller_node:main',
    ],
},

)
