from setuptools import find_packages, setup

package_name = 'taeyoung_coverage'

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
    maintainer='taeyoungkim',
    maintainer_email='goldrunty@gmail.com',
    description='Complete Coverage Path Planning for TurtleBot3 (Boustrophedon + Nav2 FollowPath)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'taeyoung_coverage_node = taeyoung_coverage.coverage_node:main'
        ],
    },
)
