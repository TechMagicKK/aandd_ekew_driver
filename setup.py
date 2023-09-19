from setuptools import setup
import os
from glob import glob

package_name = 'aandd_ekew_driver_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jiaqing Lin',
    maintainer_email='lin.jiaqing@techmagic.co.jp',
    author='koki-ogura',
    author_email='koki.balian@gmail.com',
    description='aandd_ekew_driver_py',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aandd_ekew_node = aandd_ekew_driver_py.aandd_ekew_node:main',
        ],
    },
)
