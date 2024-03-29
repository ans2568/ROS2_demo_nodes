from setuptools import setup
from glob import glob
import os
package_name = 'navigation_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ans2568@gmail.com',
    description='Navigation Node for ETRI Demo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator = navigation_node.navigator:main',
            'client = navigation_node.client:main',
            'stop = navigation_node.stop:main'
        ],
    },
)
