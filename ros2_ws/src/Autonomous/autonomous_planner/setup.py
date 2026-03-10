from setuptools import find_packages, setup
from glob import glob

package_name = 'autonomous_planner'

otherfiles = [
    ('share/' + package_name + '/launch', glob('launch/*')),
]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]+otherfiles,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raymond',
    maintainer_email='ray710mond@gmail.com',
    description='TODO: Package description',
    license='BIG RED',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'autonomous_planner_button = autonomous_planner.autonomous_planner_button:main',
            'autonomous_planner_pellets = autonomous_planner.autonomous_planner_pellets:main',
        ],
    },
)
