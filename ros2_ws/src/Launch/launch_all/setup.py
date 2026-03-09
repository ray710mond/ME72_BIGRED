from setuptools import find_packages, setup
from glob import glob

package_name = 'launch_all'

otherfiles = [
    ('share/' + package_name + '/launch', glob('launch/*')),
    # make sure the packaged parameters are installed so launch_all can find them
    ('share/' + package_name + '/config', glob('config/*')),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + otherfiles,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raymond',
    maintainer_email='ray710mond@gmail.com',
    description='TODO: Package description',
    license='BIGRED',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
