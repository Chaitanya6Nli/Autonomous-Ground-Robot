from setuptools import find_packages, setup

package_name = 'week3_assignment'

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
    maintainer='chaitanya',
    maintainer_email='chaitanyabelekar59@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller = week3_assignment.pid_controller:main', 'week3_assignment.path_planner',
        ],
    },
)
