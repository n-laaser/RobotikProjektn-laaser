from setuptools import find_packages, setup

package_name = 'linebounce'

setup(
    name=linebounce,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nlaaser',
    maintainer_email='pokekristall.fan@gmail.com',
    description='Package for making a Robot with a Camera and Laserscanner follow a line between two objects in perpetuity.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'bouncer = linebounce.bouncer:main'
        ],
    },
)
